#include "opencv_calibration.hpp"

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <numeric>

std::string CameraCalib::saveCalibOpenCV(int serial) {
  std::string filename = "mono_calibration_SN" + std::to_string(serial) + ".yml";
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  if (fs.isOpened()) {
    fs << "Size" << imageSize;
    if (disto_model_RadTan) {
      fs << "K" << K << "D" << D;
    } else {
      fs << "K" << K << "D_FE" << D;
    }
    fs.release();
    return filename;
  }
  return std::string();
}

static void printDisto(const CameraCalib& calib, std::ofstream& outfile) {
  if (calib.disto_model_RadTan) {
    size_t dist_size = calib.D.total();
    outfile << "k1 = " << calib.D.at<double>(0) << "\n";
    outfile << "k2 = " << calib.D.at<double>(1) << "\n";
    outfile << "p1 = " << calib.D.at<double>(2) << "\n";
    outfile << "p2 = " << calib.D.at<double>(3) << "\n";
    outfile << "k3 = " << calib.D.at<double>(4) << "\n";
    outfile << "k4 = " << (dist_size > 5 ? calib.D.at<double>(5) : 0.0) << "\n";
    outfile << "k5 = " << (dist_size > 6 ? calib.D.at<double>(6) : 0.0) << "\n";
    outfile << "k6 = " << (dist_size > 7 ? calib.D.at<double>(7) : 0.0) << "\n";
  } else {
    outfile << "k1 = " << calib.D.at<double>(0) << "\n";
    outfile << "k2 = " << calib.D.at<double>(1) << "\n";
    outfile << "k3 = " << calib.D.at<double>(2) << "\n";
    outfile << "k4 = " << calib.D.at<double>(3) << "\n";
  }
  outfile << "\n";
}

std::string CameraCalib::saveCalibZED(int serial, bool is_4k) {
  std::string filename = "SN" + std::to_string(serial) + "_mono.conf";
  std::ofstream outfile(filename);
  if (!outfile.is_open()) {
    std::cerr << " !!! Cannot save calibration conf file." << std::endl;
    return std::string();
  }

  if (!is_4k) {  // ZED X One GS (AR0234) — 1920×1200
    if (imageSize.height != 1200) {
      std::cout << "The resolution for calibration is not valid.\n"
                   "Use HD1200 (1920x1200) for ZED X One GS." << std::endl;
      return std::string();
    }

    outfile << "[CAM_FHD1200]\n";
    outfile << "fx = " << K.at<double>(0, 0) << "\n";
    outfile << "fy = " << K.at<double>(1, 1) << "\n";
    outfile << "cx = " << K.at<double>(0, 2) << "\n";
    outfile << "cy = " << K.at<double>(1, 2) << "\n\n";

    outfile << "[CAM_FHD]\n";
    outfile << "fx = " << K.at<double>(0, 0) << "\n";
    outfile << "fy = " << K.at<double>(1, 1) << "\n";
    outfile << "cx = " << K.at<double>(0, 2) << "\n";
    outfile << "cy = " << K.at<double>(1, 2) - 60 << "\n\n";

    outfile << "[CAM_SVGA]\n";
    outfile << "fx = " << K.at<double>(0, 0) / 2 << "\n";
    outfile << "fy = " << K.at<double>(1, 1) / 2 << "\n";
    outfile << "cx = " << K.at<double>(0, 2) / 2 << "\n";
    outfile << "cy = " << K.at<double>(1, 2) / 2 << "\n\n";

    outfile << "[DISTO]\n";
    printDisto(*this, outfile);
  } else {  // ZED X One 4K (IMX678) — 3840×2160
    if (imageSize.height != 2160) {
      std::cout << "The resolution for calibration is not valid.\n"
                   "Use 4K (3840x2160) for ZED X One 4K." << std::endl;
      return std::string();
    }

    outfile << "[CAM_4k]\n";
    outfile << "fx = " << K.at<double>(0, 0) << "\n";
    outfile << "fy = " << K.at<double>(1, 1) << "\n";
    outfile << "cx = " << K.at<double>(0, 2) << "\n";
    outfile << "cy = " << K.at<double>(1, 2) << "\n\n";

    outfile << "[CAM_QHDPLUS]\n";
    outfile << "fx = " << K.at<double>(0, 0) << "\n";
    outfile << "fy = " << K.at<double>(1, 1) << "\n";
    outfile << "cx = " << K.at<double>(0, 2) - (3840 - 3200) / 2 << "\n";
    outfile << "cy = " << K.at<double>(1, 2) - (2160 - 1800) / 2 << "\n\n";

    outfile << "[CAM_FHD]\n";
    outfile << "fx = " << K.at<double>(0, 0) / 2 << "\n";
    outfile << "fy = " << K.at<double>(1, 1) / 2 << "\n";
    outfile << "cx = " << K.at<double>(0, 2) / 2 << "\n";
    outfile << "cy = " << K.at<double>(1, 2) / 2 << "\n\n";

    outfile << "[CAM_FHD1200]\n";
    outfile << "fx = " << K.at<double>(0, 0) << "\n";
    outfile << "fy = " << K.at<double>(1, 1) << "\n";
    outfile << "cx = " << K.at<double>(0, 2) - (3840 - 1920) / 2 << "\n";
    outfile << "cy = " << K.at<double>(1, 2) - (2160 - 1200) / 2 << "\n\n";

    outfile << "[DISTO]\n";
    printDisto(*this, outfile);
  }

  outfile.close();
  return filename;
}

int calibrate(int img_count, const std::string& folder, CameraCalib& calib_data,
              const BoardConfig& board, int serial, bool is_4k,
              bool use_intrinsic_prior, double max_repr_error, bool verbose) {
  BoardDetector detector(board);
  std::vector<cv::Mat> images;
  cv::Size imageSize(0, 0);

  std::cout << std::endl
            << "Loading stored images from folder: " << folder << std::endl;

  // Count images if not provided
  if (img_count == -1) {
    std::cout << " * Counting images..." << std::endl;
    int count = 0;
    while (std::filesystem::exists(folder + "image_" + std::to_string(count) + ".png"))
      count++;
    img_count = count;
    std::cout << "   - Found " << img_count << " images." << std::endl;
  }

  for (int i = 0; i < img_count; i++) {
    std::cout << "." << std::flush;
    cv::Mat grey = cv::imread(folder + "image_" + std::to_string(i) + ".png",
                              cv::IMREAD_GRAYSCALE);
    if (!grey.empty()) {
      if (imageSize.width == 0)
        imageSize = grey.size();
      else if (grey.size() != images.back().size()) {
        std::cerr << std::endl
                  << " !!! ERROR !!! Frame #" << i
                  << " has a different size from previous frames." << std::endl;
        return EXIT_FAILURE;
      }
      images.push_back(grey);
    }
  }

  std::cout << std::endl
            << " * " << images.size() << " samples collected" << std::endl;

  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> image_points;

  std::cout << "Detecting target corners on images ("
            << (detector.isCharuco() ? "ChArUco" : "chessboard") << ")"
            << std::endl;

  for (int i = 0; i < (int)images.size(); i++) {
    std::cout << "." << std::flush;
    // Full-resolution detection: builds matching object/image points per frame
    // (a ChArUco frame may hold only a partial, id-tagged corner subset).
    BoardDetection det = detector.detect(images[i]);
    if (det.found &&
        (int)det.imagePoints.size() >= detector.minValidCorners()) {
      image_points.push_back(det.imagePoints);
      object_points.push_back(det.objectPoints);
    } else {
      std::cout << std::endl
                << "- No valid target on frame #" << i << " -" << std::endl;
    }
  }

  if ((int)image_points.size() < MIN_IMAGE) {
    std::cout << " !!! Not enough images with target detected !!!" << std::endl;
    std::cout << " Please perform a new data acquisition." << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << std::endl
            << " * Valid samples: " << image_points.size() << "/" << img_count
            << std::endl;

  auto flags = use_intrinsic_prior ? cv::CALIB_USE_INTRINSIC_GUESS : 0;

  std::cout << "Camera calibration... " << std::flush;
  double rms = -1.0;
  try {
    rms = calib_data.mono_calibrate(object_points, image_points, imageSize,
                                    flags, verbose);
  } catch (const cv::Exception& e) {
    // The fisheye solver (CALIB_CHECK_COND-style conditioning) throws on
    // ill-conditioned data such as sparse partial ChArUco views.
    std::cerr << std::endl
              << " !!! Calibration solver failed: " << e.what() << std::endl
              << " Collect more (and better-spread) views of the board, or "
                 "use the Radial-Tangential model instead of --fisheye."
              << std::endl;
    return EXIT_FAILURE;
  }
  calib_data.imageSize = imageSize;
  std::cout << "Done." << std::endl;

  // Per-frame outlier rejection (RadTan only; fisheye solvePnP is unsupported)
  if (calib_data.disto_model_RadTan) {
    std::vector<std::pair<double, int>> frame_errors;
    frame_errors.reserve(image_points.size());

    for (int i = 0; i < (int)image_points.size(); i++) {
      cv::Mat rvec, tvec;
      // solvePnP can throw (not just return false) on degenerate point sets
      // such as near-collinear partial ChArUco views — skip those frames.
      try {
        if (!cv::solvePnP(object_points[i], image_points[i],
                          calib_data.K, calib_data.D, rvec, tvec))
          continue;
      } catch (const cv::Exception&) {
        continue;
      }
      std::vector<cv::Point2f> proj;
      cv::projectPoints(object_points[i], rvec, tvec,
                        calib_data.K, calib_data.D, proj);
      const int n = (int)image_points[i].size();
      double err_sq = 0.0;
      for (int j = 0; j < n; j++) {
        double dx = proj[j].x - image_points[i][j].x;
        double dy = proj[j].y - image_points[i][j].y;
        err_sq += dx * dx + dy * dy;
      }
      frame_errors.emplace_back(std::sqrt(err_sq / n), i);
    }

    auto sorted = frame_errors;
    std::sort(sorted.begin(), sorted.end());
    // Guard against an all-solvePnP-failed frame set (empty -> no rejection).
    const double median_err =
        sorted.empty() ? 0.0 : sorted[sorted.size() / 2].first;
    const double threshold = std::max(2.5 * median_err, max_repr_error);

    std::vector<std::vector<cv::Point3f>> clean_obj;
    std::vector<std::vector<cv::Point2f>> clean_pts;
    int n_removed = 0;

    for (auto& [fe, idx] : frame_errors) {
      if (fe <= threshold) {
        clean_obj.push_back(object_points[idx]);
        clean_pts.push_back(image_points[idx]);
      } else {
        std::cout << "\n  * Removing outlier frame #" << idx
                  << " (RMS=" << std::fixed << std::setprecision(2)
                  << fe << " px, threshold=" << threshold << " px)";
        n_removed++;
      }
    }

    if (n_removed > 0) {
      std::cout << "\n * Re-running calibration after removing "
                << n_removed << " outlier frame(s)" << std::endl;
      if ((int)clean_obj.size() >= MIN_IMAGE) {
        rms = calib_data.mono_calibrate(clean_obj, clean_pts, imageSize,
                                        cv::CALIB_USE_INTRINSIC_GUESS, verbose);
        std::cout << " * RMS after outlier removal: " << rms << " px" << std::endl;
      } else {
        std::cout << " * Not enough frames left after outlier removal ("
                  << clean_obj.size() << "/" << MIN_IMAGE
                  << "). Keeping original calibration." << std::endl;
      }
    }
  }

  std::cout << std::endl << "*** Calibration Report ***" << std::endl;
  std::cout << " * Reprojection error (RMS): " << rms << " px"
            << (rms > max_repr_error ? "\t!!! TOO HIGH !!!" : "\t-> GOOD")
            << std::endl;

  if (rms > max_repr_error) {
    std::cerr << std::endl
              << "\t!!! ERROR !!!" << std::endl
              << "The reprojection error is too high (> " << max_repr_error
              << " px). Check that the lens is clean and the calibration "
                 "pattern is printed/mounted on a RIGID and FLAT surface."
              << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << std::endl << "** Camera parameters **" << std::endl;
  std::cout << "* Intrinsic matrix K:" << std::endl << calib_data.K << std::endl;
  std::cout << "* Distortion coefficients D:" << std::endl
            << calib_data.D << std::endl;

  std::cout << std::endl << "*** Save Calibration files ***" << std::endl;

  std::string yml_file = calib_data.saveCalibOpenCV(serial);
  if (!yml_file.empty())
    std::cout << " * OpenCV calibration file saved: " << yml_file << std::endl;
  else
    std::cout << " !!! Failed to save OpenCV calibration file !!!" << std::endl;

  std::string conf_file = calib_data.saveCalibZED(serial, is_4k);
  if (!conf_file.empty())
    std::cout << " * ZED conf file saved: " << conf_file << std::endl;
  else
    std::cout << " !!! Failed to save ZED conf file !!!" << std::endl;

  return EXIT_SUCCESS;
}
