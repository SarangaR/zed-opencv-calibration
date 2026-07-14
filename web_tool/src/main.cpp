// zed_calib_web — headless calibration engine for the web UI.
//
// One binary, four modes (the four X11 tools with their GUI replaced by an
// MJPEG stream). Spawned and supervised by web/server.py:
//
//   zed_calib_web --mode mono_calib   --stream-port 30001 [tool flags...]
//   zed_calib_web --mode stereo_calib --stream-port 30001 [tool flags...]
//   zed_calib_web --mode mono_check   --stream-port 30001 [tool flags...]
//   zed_calib_web --mode stereo_check --stream-port 30001 [tool flags...]
//
// Stop = SIGINT/SIGTERM (graceful: camera released, partial data kept).

#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "http_streamer.hpp"
#include "web_common.hpp"

std::atomic<bool> g_stop_requested{false};

namespace {

void onStopSignal(int sig) {
  g_stop_requested = true;
  // A second signal falls back to the default handler and force-kills, in
  // case the process is stuck outside the acquisition loop.
  std::signal(sig, SIG_DFL);
}

void printUsage(const char* app) {
  std::cout
      << "Usage: " << app
      << " --mode {mono_calib|stereo_calib|mono_check|stereo_check}\n"
         "       [--stream-port <port>] [tool flags...]\n\n"
         "Publishes the composed GUI as MJPEG on --stream-port (default "
         "30001).\nAll remaining flags are forwarded to the selected mode "
         "(same CLI as the\nstandalone tools; pass --help after --mode to see "
         "them).\n";
}

}  // namespace

int main(int argc, char* argv[]) {
  std::signal(SIGINT, onStopSignal);
  std::signal(SIGTERM, onStopSignal);

  std::string mode;
  int stream_port = 30001;
  std::vector<char*> fwd;  // argv forwarded to the mode's own parser
  fwd.push_back(argv[0]);

  for (int i = 1; i < argc; i++) {
    if (std::strcmp(argv[i], "--mode") == 0 && i + 1 < argc) {
      mode = argv[++i];
    } else if (std::strcmp(argv[i], "--stream-port") == 0 && i + 1 < argc) {
      stream_port = std::stoi(argv[++i]);
    } else if ((std::strcmp(argv[i], "--help") == 0 ||
                std::strcmp(argv[i], "-h") == 0) &&
               mode.empty()) {
      printUsage(argv[0]);
      return EXIT_SUCCESS;
    } else {
      fwd.push_back(argv[i]);
    }
  }

  if (mode.empty()) {
    printUsage(argv[0]);
    return EXIT_FAILURE;
  }

  HttpStreamer streamer;
  if (!streamer.start(stream_port)) return EXIT_FAILURE;

  const int n = static_cast<int>(fwd.size());
  int rc;
  if (mode == "mono_calib")
    rc = run_mono_calib(n, fwd.data(), streamer);
  else if (mode == "stereo_calib")
    rc = run_stereo_calib(n, fwd.data(), streamer);
  else if (mode == "mono_check")
    rc = run_mono_check(n, fwd.data(), streamer);
  else if (mode == "stereo_check")
    rc = run_stereo_check(n, fwd.data(), streamer);
  else {
    std::cerr << "Unknown --mode '" << mode << "'." << std::endl;
    printUsage(argv[0]);
    rc = EXIT_FAILURE;
  }

  streamer.stop();
  return rc;
}
