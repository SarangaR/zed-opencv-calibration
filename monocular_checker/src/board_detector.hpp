#pragma once

// Unified calibration-target detector supporting both a classic dense
// chessboard and a ChArUco board (chessboard fused with ArUco markers).
//
// ChArUco is preferred for camera calibration: because every corner is tied to
// a uniquely identified ArUco marker, partial / occluded / edge-cropped views
// are still usable (a plain chessboard is all-or-nothing), while the corners
// keep chessboard-grade subpixel accuracy.
//
// The OpenCV ChArUco API changed at 4.7 (moved from the contrib `aruco` module
// to core `objdetect`). All of that version handling is isolated inside
// board_detector.cpp; this header is version-agnostic.

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

enum class PatternType { Chessboard, Charuco };

struct BoardConfig {
  PatternType type = PatternType::Chessboard;

  // Chessboard: number of INNER corners (where black/white squares meet).
  int h_edges = 9;
  int v_edges = 6;

  // ChArUco: number of SQUARES along each axis.
  int squares_x = 15;
  int squares_y = 11;

  // Square side length in mm (both modes). For ChArUco this is the chessboard
  // square; the marker sits inside it.
  float square_size = 25.4f;

  // ChArUco marker side length in mm (must be < square_size).
  float marker_size = 11.0f;

  // ChArUco ArUco dictionary, e.g. "DICT_4X4_250".
  std::string dict_name = "DICT_4X4_250";

  // OpenCV flipped the default ChArUco marker/corner layout at 4.7. Set true if
  // the physical board was generated with pre-4.7 ("legacy") tooling; only has
  // an effect when building against OpenCV >= 4.7 (<= 4.6 is always legacy).
  bool charuco_legacy = false;
};

// Result of a single detection. For ChArUco, imagePoints[k] <-> objectPoints[k]
// <-> ids[k] describe one interpolated chessboard corner. For a chessboard,
// ids is empty and imagePoints/objectPoints hold the full ordered dense grid.
struct BoardDetection {
  bool found = false;
  std::vector<cv::Point2f> imagePoints;    // detected corners, image coords
  std::vector<cv::Point3f> objectPoints;   // matching board points (planar, Z=0)
  std::vector<int> ids;                    // ChArUco corner ids; empty for chessboard
  std::vector<cv::Point2f> outsideCorners; // 4 extreme corners TL,TR,BR,BL (coverage geometry)

  // ChArUco only: raw ArUco marker detections (4 corners per marker + id),
  // filled even when the board detection itself fails (`found == false`) so
  // the live GUI can show which tags are seen. Empty for a chessboard.
  std::vector<std::vector<cv::Point2f>> markerCorners;
  std::vector<int> markerIds;
};

class BoardDetector {
 public:
  explicit BoardDetector(const BoardConfig& cfg);
  ~BoardDetector();

  // Detect on a full-resolution image (grayscale or BGR accepted). ArUco marker
  // detection is scale-sensitive, so callers must NOT downscale before this.
  BoardDetection detect(const cv::Mat& image) const;

  // Overlay detected corners (in the coordinate system of `bgr`) for the live GUI.
  void draw(cv::Mat& bgr, const BoardDetection& det) const;

  const BoardConfig& config() const { return cfg_; }
  bool isCharuco() const { return cfg_.type == PatternType::Charuco; }

  int cornersX() const;  // ChArUco: squares_x-1 ; chessboard: h_edges
  int cornersY() const;  // ChArUco: squares_y-1 ; chessboard: v_edges

  // Minimum interpolated corners for a ChArUco frame to be usable.
  int minValidCorners() const { return isCharuco() ? 6 : cornersX() * cornersY(); }

  // 4 extreme corners (TL,TR,BR,BL) picked from an id-tagged corner subset,
  // for the coverage/skew geometry. Returns empty if fewer than 4 usable.
  std::vector<cv::Point2f> extremeCorners(
      const std::vector<cv::Point2f>& pts, const std::vector<int>& ids) const;

 private:
  BoardConfig cfg_;
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

// Map an OpenCV predefined ArUco dictionary name to its integer id.
// Returns a negative value if the name is unknown.
int arucoDictId(const std::string& name);
