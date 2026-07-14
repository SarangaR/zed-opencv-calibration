#ifndef WEB_COMMON_HPP
#define WEB_COMMON_HPP

#include <atomic>

class HttpStreamer;

// Set by the SIGINT/SIGTERM handler installed in main.cpp; every mode's
// acquisition loop polls it for graceful shutdown (camera release + cleanup).
extern std::atomic<bool> g_stop_requested;

// One entry point per tab of the web UI. argv holds the mode-specific flags
// (same CLI as the standalone tools); frames go out through the streamer.
int run_mono_calib(int argc, char** argv, HttpStreamer& streamer);
int run_stereo_calib(int argc, char** argv, HttpStreamer& streamer);
int run_mono_check(int argc, char** argv, HttpStreamer& streamer);
int run_stereo_check(int argc, char** argv, HttpStreamer& streamer);

#endif  // WEB_COMMON_HPP
