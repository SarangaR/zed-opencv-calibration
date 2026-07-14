#include "board_detector.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <set>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

// OpenCV moved the ChArUco API from the contrib `aruco` module into core
// `objdetect` at version 4.7. Select the right headers/API accordingly.
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#define CHARUCO_NEW_API 1
#include <opencv2/objdetect.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#else
#define CHARUCO_NEW_API 0
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#endif

int arucoDictId(const std::string& name) {
  static const std::map<std::string, int> table = {
      {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
      {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
      {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
      {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
      {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
      {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
      {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
      {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
      {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
      {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
      {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
      {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
      {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
      {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
      {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
      {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
      {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
      {"DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5},
      {"DICT_APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9},
      {"DICT_APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10},
      {"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11},
  };
  auto it = table.find(name);
  return it == table.end() ? -1 : it->second;
}

struct BoardDetector::Impl {
#if CHARUCO_NEW_API
  cv::aruco::CharucoBoard board;
  cv::aruco::CharucoDetector detector;
  Impl(const cv::aruco::CharucoBoard& b) : board(b), detector(b) {}
#else
  cv::Ptr<cv::aruco::CharucoBoard> board;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::Ptr<cv::aruco::DetectorParameters> params;
#endif
};

BoardDetector::BoardDetector(const BoardConfig& cfg) : cfg_(cfg) {
  if (cfg_.type != PatternType::Charuco) return;

  int dict_id = arucoDictId(cfg_.dict_name);
  if (dict_id < 0) {
    std::cerr << " !!! Unknown ArUco dictionary '" << cfg_.dict_name
              << "', falling back to DICT_4X4_250." << std::endl;
    dict_id = cv::aruco::DICT_4X4_250;
  }

#if CHARUCO_NEW_API
  cv::aruco::Dictionary dictionary =
      cv::aruco::getPredefinedDictionary(dict_id);
  cv::aruco::CharucoBoard board(cv::Size(cfg_.squares_x, cfg_.squares_y),
                                cfg_.square_size, cfg_.marker_size, dictionary);
  // Match the board's generation convention (see BoardConfig::charuco_legacy).
  board.setLegacyPattern(cfg_.charuco_legacy);
  impl_ = std::make_shared<Impl>(board);
#else
  // OpenCV <= 4.6 only implements the legacy layout.
  impl_ = std::make_shared<Impl>();
  impl_->dictionary = cv::aruco::getPredefinedDictionary(dict_id);
  impl_->board = cv::aruco::CharucoBoard::create(
      cfg_.squares_x, cfg_.squares_y, cfg_.square_size, cfg_.marker_size,
      impl_->dictionary);
  impl_->params = cv::aruco::DetectorParameters::create();
#endif
}

BoardDetector::~BoardDetector() = default;

int BoardDetector::cornersX() const {
  return isCharuco() ? cfg_.squares_x - 1 : cfg_.h_edges;
}
int BoardDetector::cornersY() const {
  return isCharuco() ? cfg_.squares_y - 1 : cfg_.v_edges;
}

std::vector<cv::Point2f> BoardDetector::extremeCorners(
    const std::vector<cv::Point2f>& pts, const std::vector<int>& ids) const {
  if (pts.size() != ids.size() || pts.size() < 4) return {};

  const int cX = cornersX();
  const int cY = cornersY();
  // Target board grid-corners: TL, TR, BR, BL.
  const int tx[4] = {0, cX - 1, cX - 1, 0};
  const int ty[4] = {0, 0, cY - 1, cY - 1};

  std::vector<cv::Point2f> out(4);
  int best_idx[4] = {-1, -1, -1, -1};
  double best_d[4] = {std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max()};

  for (size_t k = 0; k < ids.size(); k++) {
    const int gx = ids[k] % cX;
    const int gy = ids[k] / cX;
    for (int c = 0; c < 4; c++) {
      const double dx = gx - tx[c];
      const double dy = gy - ty[c];
      const double d = dx * dx + dy * dy;
      if (d < best_d[c]) {
        best_d[c] = d;
        best_idx[c] = static_cast<int>(k);
      }
    }
  }

  // Require 4 distinct corners, else the quad is degenerate.
  for (int c = 0; c < 4; c++) {
    if (best_idx[c] < 0) return {};
    for (int c2 = 0; c2 < c; c2++)
      if (best_idx[c] == best_idx[c2]) return {};
    out[c] = pts[best_idx[c]];
  }
  return out;
}

BoardDetection BoardDetector::detect(const cv::Mat& image) const {
  BoardDetection det;

  cv::Mat gray;
  if (image.channels() == 1)
    gray = image;
  else
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  if (!isCharuco()) {
    // ---- Classic chessboard (dense, all-or-nothing) ----
    const cv::Size t_size(cfg_.h_edges, cfg_.v_edges);
    std::vector<cv::Point2f> pts;
    bool found = cv::findChessboardCorners(
        gray, t_size, pts,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    if (!found) return det;

    const int win = (gray.cols >= 3000) ? 15 : 11;
    cv::cornerSubPix(
        gray, pts, cv::Size(win, win), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30,
                         0.001));

    det.found = true;
    det.imagePoints = pts;
    det.objectPoints.reserve(pts.size());
    for (int i = 0; i < cfg_.v_edges; i++)
      for (int j = 0; j < cfg_.h_edges; j++)
        det.objectPoints.emplace_back(cfg_.square_size * j,
                                      cfg_.square_size * i, 0.f);
    // Outside corners by dense index (TL,TR,BR,BL).
    det.outsideCorners = {pts[0], pts[cfg_.h_edges - 1], pts[pts.size() - 1],
                          pts[pts.size() - cfg_.h_edges]};
    return det;
  }

  // ---- ChArUco (partial views OK, id-tagged corners) ----
  std::vector<cv::Point2f> charucoCorners;
  std::vector<int> charucoIds;

#if CHARUCO_NEW_API
  impl_->detector.detectBoard(gray, charucoCorners, charucoIds,
                              det.markerCorners, det.markerIds);
#else
  cv::aruco::detectMarkers(gray, impl_->dictionary, det.markerCorners,
                           det.markerIds, impl_->params);
  if (!det.markerIds.empty()) {
    cv::aruco::interpolateCornersCharuco(det.markerCorners, det.markerIds, gray,
                                         impl_->board, charucoCorners,
                                         charucoIds);
  }
#endif

  if ((int)charucoIds.size() < 4) return det;

  const int cX = cornersX();

  // Degeneracy gate: corners confined to a single board row or column are
  // collinear — solvePnP's planar init throws on such sets, and the coverage
  // quad becomes garbage. Require spread over >= 2 rows AND >= 2 columns.
  {
    std::set<int> rows, cols;
    for (int id : charucoIds) {
      cols.insert(id % cX);
      rows.insert(id / cX);
    }
    if (rows.size() < 2 || cols.size() < 2) return det;
  }
  det.imagePoints = charucoCorners;
  det.ids = charucoIds;
  det.objectPoints.reserve(charucoIds.size());
  for (int id : charucoIds) {
    const int gx = id % cX;
    const int gy = id / cX;
    det.objectPoints.emplace_back(cfg_.square_size * gx, cfg_.square_size * gy,
                                  0.f);
  }
  det.outsideCorners = extremeCorners(charucoCorners, charucoIds);
  det.found = true;
  return det;
}

void BoardDetector::draw(cv::Mat& bgr, const BoardDetection& det) const {
  if (!isCharuco()) {
    if (!det.found) return;
    cv::drawChessboardCorners(bgr, cv::Size(cfg_.h_edges, cfg_.v_edges),
                              cv::Mat(det.imagePoints), det.found);
    return;
  }

  // Raw ArUco tag outlines, shown even when the board detection failed so the
  // operator can see exactly which markers the camera picks up.
  for (const auto& mc : det.markerCorners) {
    std::vector<cv::Point> poly(mc.begin(), mc.end());
    cv::polylines(bgr, poly, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
  }

  if (!det.found) return;

  for (size_t k = 0; k < det.imagePoints.size(); k++) {
    cv::circle(bgr, det.imagePoints[k], 4, cv::Scalar(0, 255, 255), -1);
    cv::circle(bgr, det.imagePoints[k], 5, cv::Scalar(0, 0, 0), 1);
  }
}
