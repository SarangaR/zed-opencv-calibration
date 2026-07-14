#ifndef HTTP_STREAMER_HPP
#define HTTP_STREAMER_HPP

#include <opencv2/core.hpp>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

// Minimal MJPEG-over-HTTP publisher (multipart/x-mixed-replace). The engine
// publishes the composed GUI frame each loop iteration; any number of HTTP
// clients (the web UI's <img>, VLC, a browser tab) receive it as a motion
// JPEG stream.
//
// Two kinds of requests: any path starting with "/ctrl" is a control request
// (query flags below, answered immediately); every other path streams MJPEG.
class HttpStreamer {
 public:
  ~HttpStreamer() { stop(); }

  // Bind + listen on 127.0.0.1:<port> and start the accept thread.
  bool start(int port);

  // JPEG-encode the frame and hand it to all connected clients.
  void publish(const cv::Mat& bgr);

  void stop();

  // ---- Control state, set by "GET /ctrl?..." from the web UI ----
  // /ctrl?auto=1 | auto=0   toggle auto-capture (calibration modes)
  // /ctrl?capture=1         request one manual capture attempt
  bool autoCaptureEnabled() const { return auto_capture_; }
  bool consumeCaptureRequest() { return capture_requested_.exchange(false); }

 private:
  void acceptLoop();
  void clientLoop(int fd);

  std::atomic<bool> auto_capture_{true};
  std::atomic<bool> capture_requested_{false};

  int listen_fd_ = -1;
  std::atomic<bool> running_{false};
  std::thread accept_thread_;
  std::vector<std::thread> client_threads_;
  std::mutex clients_mutex_;

  // Latest encoded frame, sequence-numbered so clients send each frame once.
  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  std::vector<uchar> frame_jpeg_;
  uint64_t frame_seq_ = 0;
};

#endif  // HTTP_STREAMER_HPP
