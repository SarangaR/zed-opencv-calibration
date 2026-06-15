#pragma once

#include <cmath>
#include <fstream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <sl/CameraOne.hpp>

constexpr int MIN_IMAGE = 20;

struct CameraCalib {
  cv::Mat K;
  cv::Mat D;
  bool disto_model_RadTan = true;
  cv::Size imageSize;

  void initDefault(bool radtan) {
    disto_model_RadTan = radtan;
    K = cv::Mat::eye(3, 3, CV_64FC1);
    if (disto_model_RadTan) {
      const int nb_coeff = 8;  // 6 radial + 2 tangential
      D = cv::Mat::zeros(1, nb_coeff, CV_64FC1);
    } else {
      // Fisheye model: k1, k2, k3, k4
      D = cv::Mat::zeros(1, 4, CV_64FC1);
    }
  }

  void setFrom(const sl::CameraParameters& cam) {
    K = cv::Mat::eye(3, 3, CV_64FC1);
    K.at<double>(0, 0) = static_cast<double>(cam.fx);
    K.at<double>(1, 1) = static_cast<double>(cam.fy);
    K.at<double>(0, 2) = static_cast<double>(cam.cx);
    K.at<double>(1, 2) = static_cast<double>(cam.cy);

    // tangential coefficients (p1, p2) are zero in Fisheye; k3/k4 are non-zero
    if (cam.disto[2] == 0. && cam.disto[3] == 0. && cam.disto[4] != 0. &&
        cam.disto[5] != 0.) {
      disto_model_RadTan = false;
      D = cv::Mat::zeros(1, 4, CV_64FC1);
      D.at<double>(0) = cam.disto[0];
      D.at<double>(1) = cam.disto[1];
      D.at<double>(2) = cam.disto[4];
      D.at<double>(3) = cam.disto[5];
    } else {
      disto_model_RadTan = true;
      const int nb_coeff = 8;
      D = cv::Mat::zeros(1, nb_coeff, CV_64FC1);
      for (int i = 0; i < nb_coeff; i++) D.at<double>(i) = cam.disto[i];
    }
  }

  // Note: object_points and image_points must be Point3f / Point2f.
  // Point3d / Point2d are not supported by cv::calibrateCamera.
  double mono_calibrate(const std::vector<std::vector<cv::Point3f>>& object_points,
                        const std::vector<std::vector<cv::Point2f>>& image_points,
                        const cv::Size& image_size, int flags, bool verbose) {
    double rms = -1.0;
    std::vector<cv::Mat> rvec, tvec;
    if (disto_model_RadTan) {
      if (D.cols >= 8) {
        flags += cv::CALIB_RATIONAL_MODEL;
        if (verbose)
          std::cout << "[DEBUG] Using Rational model (8 distortion coefficients)." << std::endl;
      }
      if (verbose)
        std::cout << "[DEBUG] Calibrating with Radial-Tangential model..." << std::endl;
      rms = cv::calibrateCamera(object_points, image_points, image_size, K, D,
                                rvec, tvec, flags);
    } else {
      if (verbose)
        std::cout << "[DEBUG] Calibrating with Fisheye model..." << std::endl;
      rms = cv::fisheye::calibrate(
          object_points, image_points, image_size, K, D, rvec, tvec,
          flags + cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC +
              cv::fisheye::CALIB_FIX_SKEW);
    }

    if (verbose) {
      std::cout << "[DEBUG] K:" << std::endl << K << std::endl;
      std::cout << "[DEBUG] D:" << std::endl << D << std::endl;
      std::cout << "[DEBUG] RMS: " << rms << std::endl;
    }
    return rms;
  }

  std::string saveCalibOpenCV(int serial);
  std::string saveCalibZED(int serial, bool is_4k);
};

int calibrate(int img_count, const std::string& folder, CameraCalib& calib_data,
              int h_edges, int v_edges, double square_size, int serial,
              bool is_4k, bool use_intrinsic_prior = false,
              double max_repr_error = 0.5, bool verbose = false);
