#!/usr/bin/env python3
"""Web launcher for the ZED calibration toolkit (stdlib only).

Serves the tab UI on --port, spawns/supervises the zed_calib_web engine
(one run at a time -- one camera), and proxies its MJPEG stream:

    GET  /            index.html
    POST /api/start   {"mode": "...", "args": ["--charuco", ...]}
    POST /api/stop    SIGINT the engine (graceful), SIGKILL after 5 s
    GET  /api/status  {"running": bool, "mode": str|null, "exit_code": int|null}
    GET  /api/stream  MJPEG proxied from the engine (127.0.0.1:--stream-port)
    POST /api/upload?name=x.yml   save raw body to <workdir>/uploads/x.yml
    GET  /api/files               list *.yml / *.conf in the workdir
    GET  /api/download?name=x.yml serve a workdir output file as attachment
"""

import argparse
import glob
import http.client
import json
import os
import signal
import socket
import subprocess
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MODES = ("mono_calib", "stereo_calib", "mono_check", "stereo_check")

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")


class Engine:
    """The single active zed_calib_web process."""

    def __init__(self, binary, stream_port, workdir):
        self.binary = binary
        self.stream_port = stream_port
        self.workdir = workdir
        self.lock = threading.Lock()
        self.proc = None
        self.mode = None
        self.exit_code = None

    def start(self, mode, args):
        with self.lock:
            if self.proc is not None and self.proc.poll() is None:
                return False, "already running"
            cmd = [self.binary, "--mode", mode,
                   "--stream-port", str(self.stream_port)] + args
            print(">>> start:", " ".join(cmd), flush=True)
            self.proc = subprocess.Popen(cmd, cwd=self.workdir)
            self.mode = mode
            self.exit_code = None
            return True, "started"

    def stop(self):
        with self.lock:
            proc = self.proc
        if proc is None or proc.poll() is not None:
            return "not running"
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            print(">>> engine did not exit after SIGINT, killing", flush=True)
            proc.kill()
            proc.wait()
        return "stopped"

    def status(self):
        with self.lock:
            if self.proc is None:
                return {"running": False, "mode": None, "exit_code": None}
            code = self.proc.poll()
            if code is not None:
                self.exit_code = code
            return {"running": code is None, "mode": self.mode,
                    "exit_code": self.exit_code}


ENGINE = None  # set in main()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # quiet the per-request spam
        if "/api/stream" not in getattr(self, "path", ""):
            super().log_message(fmt, *args)

    # ---- helpers ----
    def send_json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    # ---- routes ----
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            try:
                with open(os.path.join(STATIC_DIR, "index.html"), "rb") as f:
                    body = f.read()
            except OSError:
                self.send_error(500, "index.html missing")
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/api/status":
            self.send_json(ENGINE.status())
        elif self.path.startswith("/api/stream"):
            self.proxy_stream()
        elif self.path.startswith("/api/ctrl"):
            self.proxy_ctrl()
        elif self.path == "/api/files":
            self.list_files()
        elif self.path.startswith("/api/download"):
            self.download_file()
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == "/api/start":
            length = int(self.headers.get("Content-Length", 0))
            try:
                req = json.loads(self.rfile.read(length) or b"{}")
            except json.JSONDecodeError:
                self.send_json({"error": "bad json"}, 400)
                return
            mode = req.get("mode")
            args = req.get("args", [])
            if mode not in MODES:
                self.send_json({"error": f"mode must be one of {MODES}"}, 400)
                return
            if (not isinstance(args, list)
                    or not all(isinstance(a, str) for a in args)):
                self.send_json({"error": "args must be a list of strings"}, 400)
                return
            ok, msg = ENGINE.start(mode, args)
            self.send_json({"ok": ok, "message": msg}, 200 if ok else 409)
        elif self.path == "/api/stop":
            self.send_json({"ok": True, "message": ENGINE.stop()})
        elif self.path.startswith("/api/upload"):
            self.upload_file()
        else:
            self.send_error(404)

    # ---- calibration-file upload / output download ----
    MAX_UPLOAD = 16 * 1024 * 1024

    def query_name(self):
        """Sanitized 'name' query parameter, or None."""
        query = urllib.parse.urlparse(self.path).query
        name = urllib.parse.parse_qs(query).get("name", [""])[0]
        name = os.path.basename(name.replace("\\", "/"))
        return name or None

    def upload_file(self):
        name = self.query_name()
        if not name or not name.lower().endswith((".yml", ".yaml")):
            self.send_json({"error": "name must be a .yml/.yaml file"}, 400)
            return
        length = int(self.headers.get("Content-Length", 0))
        if length <= 0 or length > self.MAX_UPLOAD:
            self.send_json({"error": "bad upload size"}, 400)
            return
        updir = os.path.join(ENGINE.workdir, "uploads")
        os.makedirs(updir, exist_ok=True)
        dest = os.path.join(updir, name)
        with open(dest, "wb") as f:
            remaining = length
            while remaining:
                chunk = self.rfile.read(min(65536, remaining))
                if not chunk:
                    break
                f.write(chunk)
                remaining -= len(chunk)
        if remaining:
            os.unlink(dest)
            self.send_json({"error": "truncated upload"}, 400)
            return
        self.send_json({"ok": True, "path": "uploads/" + name})

    def list_files(self):
        entries = []
        for pattern in ("*.yml", "*.conf"):
            for p in glob.glob(os.path.join(ENGINE.workdir, pattern)):
                st = os.stat(p)
                entries.append({"name": os.path.basename(p),
                                "size": st.st_size,
                                "mtime": int(st.st_mtime)})
        entries.sort(key=lambda e: e["mtime"], reverse=True)
        self.send_json({"files": entries})

    def download_file(self):
        name = self.query_name()
        if not name or not name.lower().endswith((".yml", ".conf")):
            self.send_error(404)
            return
        path = os.path.join(ENGINE.workdir, name)
        if not os.path.isfile(path):
            self.send_error(404)
            return
        with open(path, "rb") as f:
            body = f.read()
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Disposition",
                         f'attachment; filename="{name}"')
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def proxy_ctrl(self):
        """Forward a control request (auto-capture toggle, manual capture)
        to the engine's /ctrl endpoint."""
        query = self.path.split("?", 1)[1] if "?" in self.path else ""
        try:
            conn = http.client.HTTPConnection("127.0.0.1", ENGINE.stream_port,
                                              timeout=3)
            conn.request("GET", "/ctrl?" + query)
            conn.getresponse().read()
            conn.close()
            self.send_json({"ok": True})
        except OSError:
            self.send_json({"ok": False, "error": "engine not running"}, 503)

    def proxy_stream(self):
        """Relay the engine's MJPEG stream byte-for-byte to this client."""
        upstream = None
        deadline = time.time() + 10  # engine may still be binding its port
        while time.time() < deadline:
            try:
                upstream = socket.create_connection(
                    ("127.0.0.1", ENGINE.stream_port), timeout=3)
                break
            except OSError:
                if not ENGINE.status()["running"]:
                    break
                time.sleep(0.3)
        if upstream is None:
            self.send_error(503, "engine stream not available")
            return
        try:
            upstream.sendall(b"GET /stream HTTP/1.0\r\n\r\n")
            upstream.settimeout(10)
            # Skip the upstream response headers; send our own.
            buf = b""
            while b"\r\n\r\n" not in buf:
                chunk = upstream.recv(1024)
                if not chunk:
                    self.send_error(502)
                    return
                buf += chunk
            _, rest = buf.split(b"\r\n\r\n", 1)

            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=zedframe")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            if rest:
                self.wfile.write(rest)
            while True:
                chunk = upstream.recv(65536)
                if not chunk:
                    break
                self.wfile.write(chunk)
        except OSError:
            pass  # client or engine went away -- normal
        finally:
            upstream.close()
            self.close_connection = True


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=30000)
    ap.add_argument("--engine", required=True,
                    help="path to the zed_calib_web binary")
    ap.add_argument("--stream-port", type=int, default=30001,
                    help="internal port the engine publishes MJPEG on")
    ap.add_argument("--workdir", default=os.getcwd(),
                    help="engine working directory (calibration outputs land "
                         "here)")
    opts = ap.parse_args()

    global ENGINE
    ENGINE = Engine(os.path.abspath(opts.engine), opts.stream_port,
                    os.path.abspath(opts.workdir))

    server = ThreadingHTTPServer(("0.0.0.0", opts.port), Handler)
    server.daemon_threads = True
    print(f"*** ZED calibration web UI on http://0.0.0.0:{opts.port} ***",
          flush=True)

    def shutdown(signum, frame):
        print(">>> shutting down", flush=True)
        ENGINE.stop()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    server.serve_forever()


if __name__ == "__main__":
    main()
