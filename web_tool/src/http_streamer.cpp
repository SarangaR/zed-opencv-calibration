#include "http_streamer.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <string>
#include <opencv2/imgcodecs.hpp>

namespace {

const char* kStreamHeader =
    "HTTP/1.0 200 OK\r\n"
    "Connection: close\r\n"
    "Cache-Control: no-cache, no-store\r\n"
    "Pragma: no-cache\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=zedframe\r\n"
    "\r\n";

bool sendAll(int fd, const void* data, size_t len) {
  const char* p = static_cast<const char*>(data);
  while (len > 0) {
    ssize_t n = ::send(fd, p, len, MSG_NOSIGNAL);
    if (n <= 0) return false;
    p += n;
    len -= static_cast<size_t>(n);
  }
  return true;
}

}  // namespace

bool HttpStreamer::start(int port) {
  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    std::cerr << "HttpStreamer: socket() failed" << std::endl;
    return false;
  }
  int one = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);  // launcher proxies; docker-net OK
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) <
      0) {
    std::cerr << "HttpStreamer: cannot bind port " << port << std::endl;
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  if (::listen(listen_fd_, 4) < 0) {
    std::cerr << "HttpStreamer: listen() failed" << std::endl;
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }

  running_ = true;
  accept_thread_ = std::thread(&HttpStreamer::acceptLoop, this);
  std::cout << " * MJPEG stream on port " << port << std::endl;
  return true;
}

void HttpStreamer::publish(const cv::Mat& bgr) {
  if (!running_ || bgr.empty()) return;
  std::vector<uchar> jpeg;
  cv::imencode(".jpg", bgr, jpeg, {cv::IMWRITE_JPEG_QUALITY, 80});
  {
    std::lock_guard<std::mutex> lk(frame_mutex_);
    frame_jpeg_ = std::move(jpeg);
    ++frame_seq_;
  }
  frame_cv_.notify_all();
}

void HttpStreamer::stop() {
  if (!running_.exchange(false)) return;
  // Unblock accept() and waiting clients.
  if (listen_fd_ >= 0) {
    ::shutdown(listen_fd_, SHUT_RDWR);
    ::close(listen_fd_);
    listen_fd_ = -1;
  }
  frame_cv_.notify_all();
  if (accept_thread_.joinable()) accept_thread_.join();
  std::lock_guard<std::mutex> lk(clients_mutex_);
  for (auto& t : client_threads_)
    if (t.joinable()) t.join();
  client_threads_.clear();
}

void HttpStreamer::acceptLoop() {
  while (running_) {
    int fd = ::accept(listen_fd_, nullptr, nullptr);
    if (fd < 0) {
      if (running_) continue;
      break;
    }
    int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    std::lock_guard<std::mutex> lk(clients_mutex_);
    client_threads_.emplace_back(&HttpStreamer::clientLoop, this, fd);
  }
}

void HttpStreamer::clientLoop(int fd) {
  // Read the request line ("GET <path> HTTP/1.x"); trailing headers ignored.
  char buf[1024] = {0};
  ssize_t got = ::recv(fd, buf, sizeof(buf) - 1, 0);
  std::string path;
  if (got > 0) {
    std::string req(buf, static_cast<size_t>(got));
    size_t sp1 = req.find(' ');
    size_t sp2 = (sp1 == std::string::npos) ? std::string::npos
                                            : req.find(' ', sp1 + 1);
    if (sp2 != std::string::npos) path = req.substr(sp1 + 1, sp2 - sp1 - 1);
  }

  // Control endpoint: flip flags and answer immediately (no stream).
  if (path.rfind("/ctrl", 0) == 0) {
    if (path.find("auto=1") != std::string::npos) auto_capture_ = true;
    if (path.find("auto=0") != std::string::npos) auto_capture_ = false;
    if (path.find("capture=1") != std::string::npos) capture_requested_ = true;
    const char resp[] =
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: 2\r\n\r\nok";
    sendAll(fd, resp, sizeof(resp) - 1);
    ::close(fd);
    return;
  }

  if (!sendAll(fd, kStreamHeader, std::strlen(kStreamHeader))) {
    ::close(fd);
    return;
  }

  uint64_t last_seq = 0;
  std::vector<uchar> jpeg;
  while (running_) {
    {
      std::unique_lock<std::mutex> lk(frame_mutex_);
      frame_cv_.wait_for(lk, std::chrono::milliseconds(500), [&] {
        return !running_ || frame_seq_ != last_seq;
      });
      if (!running_) break;
      if (frame_seq_ == last_seq) continue;  // timeout, no new frame
      last_seq = frame_seq_;
      jpeg = frame_jpeg_;
    }
    char part[128];
    int n = std::snprintf(part, sizeof(part),
                          "--zedframe\r\n"
                          "Content-Type: image/jpeg\r\n"
                          "Content-Length: %zu\r\n\r\n",
                          jpeg.size());
    if (!sendAll(fd, part, static_cast<size_t>(n)) ||
        !sendAll(fd, jpeg.data(), jpeg.size()) || !sendAll(fd, "\r\n", 2))
      break;  // client gone
  }
  ::close(fd);
}
