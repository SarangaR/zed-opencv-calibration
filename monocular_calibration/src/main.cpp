#include <atomic>
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include "board_detector.hpp"
#include "calibration_checker.hpp"
#include "opencv_calibration.hpp"

namespace fs = std::filesystem;

// *********************************************************************************
// CHANGE THIS PARAMS USING THE COMMAND LINE OPTIONS
// Learn more:
// * https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html

int h_edges = 9;           // number of horizontal inner edges
int v_edges = 6;           // number of vertical inner edges
float square_size = 25.4;  // mm (chessboard default; ChArUco defaults to 15)
bool square_size_set = false;  // true when --square_size was passed explicitly

// ChArUco board (enabled with --charuco). Defaults match the AndyMark board.
bool use_charuco = false;
int squares_x = 15;                    // number of squares along X
int squares_y = 11;                    // number of squares along Y
float marker_size = 11.0f;             // ArUco marker size in mm
std::string dict_name = "DICT_4X4_250";  // ArUco dictionary
bool charuco_legacy = false;           // pre-4.7 ChArUco board layout

// Default parameters are good for this checkerboard:
// https://github.com/opencv/opencv/blob/4.x/doc/pattern.png/
// *********************************************************************************

std::string image_folder = "zed-images/";

// Coverage indicator fill helpers
void addNewCheckerboardPosition(cv::Mat& coverage_indicator,
                                cv::Mat& pos_indicator,
                                cv::Mat& limits_indicator, float norm_x,
                                float norm_y, float norm_size, float min_x,
                                float max_x, float min_y, float max_y,
                                bool draw_rect);
void addNewCheckerboardPoly(cv::Mat& coverage_indicator,
                            const std::vector<cv::Point2f>& pts);
void applyCoverageIndicatorOverlay(cv::Mat& image,
                                   const cv::Mat& coverage_indicator,
                                   const cv::Mat& limits_indicator);
void applyPosIndicatorOverlay(cv::Mat& image, const cv::Mat& pos_indicator);

/// Rendering
constexpr int text_area_width = 750;
cv::Size display_size(720, 404);  // 16:9 default; width follows source aspect

/// Calibration condition
const float max_repr_error = 0.5f;
const int min_samples = 25;
const int max_samples = 35;
const float min_avg_x_coverage = 0.65f;
const float min_avg_y_coverage = 0.65f;
const float min_area_range = 0.38f;
const float min_skew_range = 0.335f;
const float min_b_x_coverage = 0.8f;
const float min_b_y_coverage = 0.8f;

const float min_target_area = 0.02f;
const double min_sharpness = 100.0;

// Debug
bool verbose = false;
int sdk_verbose = 0;

// Ctrl+C / SIGTERM: request a clean shutdown (release camera, exit). A second
// signal falls back to the default handler and force-kills, in case the
// process is stuck outside the acquisition loop.
std::atomic<bool> g_stop_requested{false};
void onStopSignal(int sig) {
  g_stop_requested = true;
  std::signal(sig, SIG_DFL);
}

// Auto-capture toggle: clickable button in the side panel (also key 'a').
// When enabled, any detected board is fed to CalibrationChecker::testSample
// automatically; frames are saved only when the checker accepts them, i.e.
// when they actually widen the coverage / size / skew diversity.
struct AutoCaptureButton {
  bool enabled = false;
  cv::Rect rect;  // hit area, in rendering-image (display + panel) coords
};

void onMouseToggleAutoCapture(int event, int x, int y, int /*flags*/,
                              void* userdata) {
  auto* btn = static_cast<AutoCaptureButton*>(userdata);
  if (event == cv::EVENT_LBUTTONDOWN && btn->rect.contains(cv::Point(x, y)))
    btn->enabled = !btn->enabled;
}

void scaleKP(std::vector<cv::Point2f>& pts, cv::Size in, cv::Size out) {
  float rx = out.width / static_cast<float>(in.width);
  float ry = out.height / static_cast<float>(in.height);
  for (auto& it : pts) {
    it.x *= rx;
    it.y *= ry;
  }
}

struct Args {
  std::string app_name;
  std::string svo_path = "";
  bool is_radtan_lens = true;
  int camera_id = -1;
  int camera_sn = -1;
  bool use_stored_images = false;
  bool use_webcam = false;         // test mode: plain cv::VideoCapture input
  std::string webcam_source = "0"; // device id ("0") or video file path/URL
  bool mirror = false;             // horizontally mirror the live preview only

  void parse(int argc, char* argv[]) {
    app_name = argv[0];
    for (int i = 1; i < argc; i++) {
      std::string arg = argv[i];
      if (arg == "--svo" && i + 1 < argc) {
        svo_path = argv[++i];
      } else if (arg == "--webcam" && i + 1 < argc) {
        use_webcam = true;
        webcam_source = argv[++i];
      } else if (arg == "--mirror") {
        mirror = true;
      } else if (arg == "--fisheye") {
        is_radtan_lens = false;
      } else if (arg == "--id" && i + 1 < argc) {
        camera_id = std::stoi(argv[++i]);
      } else if (arg == "--sn" && i + 1 < argc) {
        camera_sn = std::stoi(argv[++i]);
      } else if (arg == "--h_edges" && i + 1 < argc) {
        h_edges = std::stoi(argv[++i]);
      } else if (arg == "--v_edges" && i + 1 < argc) {
        v_edges = std::stoi(argv[++i]);
      } else if (arg == "--square_size" && i + 1 < argc) {
        square_size = std::stof(argv[++i]);
        square_size_set = true;
      } else if (arg == "--charuco") {
        use_charuco = true;
      } else if (arg == "--squares_x" && i + 1 < argc) {
        squares_x = std::stoi(argv[++i]);
      } else if (arg == "--squares_y" && i + 1 < argc) {
        squares_y = std::stoi(argv[++i]);
      } else if (arg == "--marker_size" && i + 1 < argc) {
        marker_size = std::stof(argv[++i]);
      } else if (arg == "--dict" && i + 1 < argc) {
        dict_name = argv[++i];
      } else if (arg == "--charuco_legacy") {
        charuco_legacy = true;
      } else if (arg == "--use_stored_values") {
        use_stored_images = true;
      } else if (arg == "--help" || arg == "-h") {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        std::cout << "  --h_edges <value>      Number of horizontal inner "
                     "edges of the checkerboard"
                  << std::endl;
        std::cout << "  --v_edges <value>      Number of vertical inner edges "
                     "of the checkerboard"
                  << std::endl;
        std::cout << "  --square_size <value>  Size of a square in mm "
                     "(default 25.4 chessboard / 15 ChArUco)"
                  << std::endl;
        std::cout << "  --charuco              Use a ChArUco board instead of a "
                     "plain chessboard"
                  << std::endl;
        std::cout << "  --squares_x <value>    ChArUco: number of squares "
                     "along X (default 15)"
                  << std::endl;
        std::cout << "  --squares_y <value>    ChArUco: number of squares "
                     "along Y (default 11)"
                  << std::endl;
        std::cout << "  --marker_size <value>  ChArUco: ArUco marker size in "
                     "mm (default 11)"
                  << std::endl;
        std::cout << "  --dict <name>          ChArUco: ArUco dictionary "
                     "(default DICT_4X4_250)"
                  << std::endl;
        std::cout << "  --charuco_legacy       ChArUco: board uses the pre-4.7 "
                     "(legacy) layout"
                  << std::endl;
        std::cout << "  --svo <file>           Path to the SVO file."
                  << std::endl;
        std::cout << "  --webcam <id|path>     Test mode: use a plain webcam "
                     "(device id) or a video file instead of a ZED."
                  << std::endl;
        std::cout << "  --mirror               Mirror the live preview "
                     "horizontally (selfie view; display only, calibration "
                     "data is unaffected)."
                  << std::endl;
        std::cout << "  --fisheye              Use fisheye lens model."
                  << std::endl;
        std::cout << "  --id <id>              Camera ID of the ZED X One."
                  << std::endl;
        std::cout << "  --sn <sn>              Serial number of the ZED X One."
                  << std::endl;
        std::cout << "  --help, -h             Show this help message."
                  << std::endl
                  << std::endl;
        std::cout << "Examples:" << std::endl;
        std::cout << std::endl
                  << "* ZED X One using default (first) camera:" << std::endl;
        std::cout << "  " << argv[0] << std::endl;
        std::cout << std::endl
                  << "* ZED X One using an SVO file:" << std::endl;
        std::cout << "  " << argv[0] << " --svo camera.svo" << std::endl;
        std::cout << std::endl
                  << "* ZED X One selected by serial number:" << std::endl;
        std::cout << "  " << argv[0] << " --sn 12345678" << std::endl;
        std::cout << std::endl
                  << "* ZED X One with fisheye lens model and custom checkerboard:"
                  << std::endl;
        std::cout << "  " << argv[0]
                  << " --fisheye --h_edges 12 --v_edges 9 --square_size 30.0"
                  << std::endl;
        std::cout << std::endl
                  << "* ZED X One with a ChArUco board (AndyMark defaults):"
                  << std::endl;
        std::cout << "  " << argv[0]
                  << " --charuco --squares_x 15 --squares_y 11 "
                     "--square_size 15 --marker_size 11 --dict DICT_4X4_250"
                  << std::endl;
        std::cout << std::endl;
        exit(0);
      }
    }
  }
};

int main(int argc, char* argv[]) {
  std::signal(SIGINT, onStopSignal);
  std::signal(SIGTERM, onStopSignal);

  const DetectedBoardParams idealParams = {
      cv::Point2f(min_avg_x_coverage, min_avg_y_coverage), min_area_range,
      min_skew_range, min_b_x_coverage, min_b_y_coverage};

  bool is_4k_camera = false;

  Args args;
  args.parse(argc, argv);

  // The 25.4 mm square default is a chessboard value; the ChArUco defaults
  // match the AndyMark board (15 mm squares). Using 25.4 with ChArUco would
  // silently scale all metric outputs by 25.4/15.
  if (use_charuco && !square_size_set) square_size = 15.0f;

  // Board configuration (chessboard by default, ChArUco with --charuco).
  BoardConfig board_cfg;
  board_cfg.type = use_charuco ? PatternType::Charuco : PatternType::Chessboard;
  board_cfg.h_edges = h_edges;
  board_cfg.v_edges = v_edges;
  board_cfg.squares_x = squares_x;
  board_cfg.squares_y = squares_y;
  board_cfg.square_size = square_size;
  board_cfg.marker_size = marker_size;
  board_cfg.dict_name = dict_name;
  board_cfg.charuco_legacy = charuco_legacy;

  BoardDetector detector(board_cfg);

  std::cout << "*** Monocular Camera Calibration Tool (ZED X One) ***" << std::endl;
  std::cout << std::endl;
  if (use_charuco) {
    std::cout << "The calibration process requires a ChArUco board of known "
                 "characteristics."
              << std::endl;
    std::cout << " * Expected ChArUco board features:" << std::endl;
    std::cout << "   - Squares (X x Y):\t\t" << squares_x << " x " << squares_y
              << std::endl;
    std::cout << "   - Square size:\t\t" << square_size << " mm" << std::endl;
    std::cout << "   - Marker size:\t\t" << marker_size << " mm" << std::endl;
    std::cout << "   - ArUco dictionary:\t\t" << dict_name << std::endl;
  } else {
    std::cout << "The calibration process requires a checkerboard of known "
                 "characteristics."
              << std::endl;
    std::cout << " * Expected checkerboard features:" << std::endl;
    std::cout << "   - Inner horizontal edges:\t" << h_edges << std::endl;
    std::cout << "   - Inner vertical edges:\t" << v_edges << std::endl;
    std::cout << "   - Square size:\t\t" << square_size << " mm" << std::endl;
  }
  std::cout << "Change these parameters using the command line options if "
               "needed. Use the '-h' option for help."
            << std::endl;
  std::cout << std::endl;

  // The checker works on the grid of chessboard corners: (h_edges, v_edges) for
  // a chessboard, (squares_x-1, squares_y-1) for a ChArUco board.
  CalibrationChecker checker(
      cv::Size(detector.cornersX(), detector.cornersY()), square_size,
      min_samples, max_samples, min_target_area, idealParams, verbose,
      use_charuco);

  CameraCalib calib;
  calib.initDefault(args.is_radtan_lens);
  std::cout << " * Lens distortion model: "
            << (args.is_radtan_lens ? "Radial-Tangential" : "Fisheye")
            << std::endl;

  int image_count = -1;
  bool can_use_calib_prior = false;
  int serial_number = 0;  // 0 in webcam test mode

  if (!args.use_stored_images) {
    float size_score = 0.0f, skew_score = 0.0f, pos_score_x = 0.0f,
          pos_score_y = 0.0f, min_bx = 0.0f, max_bx = 0.0f, min_by = 0.0f,
          max_by = 0.0f, min_size = 0.0f, max_size = 0.0f, min_skew = 0.0f,
          max_skew = 0.0f;

    // Frame source: either a ZED X One (default) or, in test mode (--webcam),
    // a plain cv::VideoCapture device / video file.
    cv::VideoCapture cap;
    cv::Size cam_size;
    cv::Mat rgb;  // current full-resolution BGR frame
#ifndef WEBCAM_ONLY
    sl::CameraOne zed_cam;
    sl::Mat zed_image;
#endif

    if (args.use_webcam) {
      const bool is_id =
          !args.webcam_source.empty() &&
          args.webcam_source.find_first_not_of("0123456789") ==
              std::string::npos;
      const bool opened = is_id ? cap.open(std::stoi(args.webcam_source))
                                : cap.open(args.webcam_source);
      if (!opened || !cap.isOpened()) {
        std::cerr << "Error opening webcam source '" << args.webcam_source
                  << "'." << std::endl;
        return EXIT_FAILURE;
      }
      if (!cap.read(rgb) || rgb.empty()) {
        std::cerr << "Webcam source '" << args.webcam_source
                  << "' produced no frame." << std::endl;
        return EXIT_FAILURE;
      }
      cam_size = rgb.size();
      std::cout << " * Webcam test mode. Source: " << args.webcam_source
                << " (" << cam_size.width << " x " << cam_size.height << ")"
                << std::endl;
      std::cout << " * Output serial number: 0 (mono_calibration_SN0.yml)"
                << std::endl;
    } else {
#ifdef WEBCAM_ONLY
      std::cerr << "This binary was built without the ZED SDK (WEBCAM_ONLY). "
                   "Use --webcam <id-or-path>."
                << std::endl;
      return EXIT_FAILURE;
#else
      sl::InitParametersOne init_params;
      init_params.camera_resolution = sl::RESOLUTION::AUTO;
      init_params.camera_fps = 15;
      init_params.sdk_verbose = sdk_verbose;

      if (!args.svo_path.empty()) {
        init_params.input.setFromSVOFile(args.svo_path.c_str());
        std::cout << " * Using SVO file: " << args.svo_path << std::endl;
      } else if (args.camera_sn != -1) {
        init_params.input.setFromSerialNumber(args.camera_sn);
        std::cout << " * Using camera serial number: " << args.camera_sn << std::endl;
      } else if (args.camera_id != -1) {
        init_params.input.setFromCameraID(args.camera_id);
        std::cout << " * Using camera ID: " << args.camera_id << std::endl;
      }

      auto status = zed_cam.open(init_params);

      if (status > sl::ERROR_CODE::SUCCESS &&
          status != sl::ERROR_CODE::INVALID_CALIBRATION_FILE) {
        std::cerr << "Error opening ZED X One camera: " << sl::toString(status)
                  << std::endl;
        return EXIT_FAILURE;
      }

      sl::CameraOneInformation zed_info = zed_cam.getCameraInformation();
      serial_number = zed_info.serial_number;

      std::cout << " * Camera Model: " << sl::toString(zed_info.camera_model)
                << std::endl;
      std::cout << " * Camera Serial Number: " << zed_info.serial_number
                << std::endl;
      std::cout << " * Camera Resolution: "
                << zed_info.camera_configuration.resolution.width << " x "
                << zed_info.camera_configuration.resolution.height << std::endl;

      is_4k_camera = (zed_info.camera_model == sl::MODEL::ZED_XONE_UHD);

      can_use_calib_prior = (status != sl::ERROR_CODE::INVALID_CALIBRATION_FILE);
      std::cout << " * Using prior calibration: "
                << (can_use_calib_prior ? "Yes" : "No") << std::endl;

      if (can_use_calib_prior)
        calib.setFrom(zed_info.camera_configuration.calibration_parameters_raw);

      sl::Resolution camera_resolution = zed_info.camera_configuration.resolution;
      cam_size = cv::Size(static_cast<int>(camera_resolution.width),
                          static_cast<int>(camera_resolution.height));

      zed_image.alloc(camera_resolution, sl::MAT_TYPE::U8_C3, sl::MEM::CPU);
      rgb = cv::Mat(cam_size.height, cam_size.width, CV_8UC3,
                    zed_image.getPtr<sl::uchar1>());
#endif
    }

    // Match the preview aspect to the source so the image is not stretched
    // (e.g. 4:3 webcams vs the 16:9 default).
    display_size.width =
        display_size.height * cam_size.width / cam_size.height;

    cv::Mat coverage_indicator =
        cv::Mat::zeros(display_size.height, display_size.width, CV_8UC1);
    cv::Mat pos_indicator =
        cv::Mat::zeros(display_size.height, display_size.width, CV_8UC1);
    cv::Mat limits_indicator =
        cv::Mat::zeros(display_size.height, display_size.width, CV_8UC1);

    cv::Mat rgb_d, rgb_d_fill, display, rendering_image;

    bool acquisition_completed = false;

    if (fs::exists(image_folder)) {
      if (verbose) {
        std::uintmax_t n{fs::remove_all(image_folder)};
        std::cout << "[DEBUG] Removed " << n
                  << " files from previous calibration." << std::endl;
      } else {
        fs::remove_all(image_folder);
      }
    }
    if (!fs::create_directories(image_folder)) {
      std::cerr << "Error creating storage folder!" << std::endl;
      return EXIT_FAILURE;
    }

    auto computeSharpness = [](const cv::Mat& bgr) -> double {
      cv::Mat grey, lap;
      cv::cvtColor(bgr, grey, cv::COLOR_BGR2GRAY);
      cv::Laplacian(grey, lap, CV_64F);
      cv::Scalar mean, stddev;
      cv::meanStdDev(lap, mean, stddev);
      return stddev.val[0] * stddev.val[0];
    };

    char key = ' ';
    bool missing_target = false;
    bool low_target_variability = false;
    bool blurry_image = false;

    const std::string window_name = "ZED X One Calibration";
    cv::namedWindow(window_name, cv::WINDOW_KEEPRATIO);
    cv::resizeWindow(window_name, display_size.width + text_area_width,
                     display_size.height);

    // Auto-capture button lives in the side panel, below the status table.
    const cv::Rect auto_btn_local(10, display_size.height - 46, 300, 36);
    AutoCaptureButton auto_btn;
    auto_btn.rect = cv::Rect(display_size.width + auto_btn_local.x,
                             auto_btn_local.y, auto_btn_local.width,
                             auto_btn_local.height);
    cv::setMouseCallback(window_name, onMouseToggleAutoCapture, &auto_btn);
    double next_auto_attempt = 0.0;  // seconds; throttles auto testSample calls
    bool window_was_visible = false;  // arms the title-bar-X close detection

    while (1) {
      if (key == 'q' || key == 'Q' || key == 27 || g_stop_requested) {
        std::cout << "Calibration aborted by user." << std::endl;
        if (args.use_webcam) cap.release();
#ifndef WEBCAM_ONLY
        if (!args.use_webcam) zed_cam.close();
#endif
        cv::destroyAllWindows();
        return EXIT_SUCCESS;
      }

      const cv::Scalar info_color = cv::Scalar(50, 210, 50);
      const cv::Scalar warn_color = cv::Scalar(0, 50, 250);

      // Grab the next full-resolution frame from the active source.
      bool frame_ok = false;
      if (args.use_webcam) {
        frame_ok = cap.read(rgb) && !rgb.empty();
        if (!frame_ok) {
          // Video file ended or the device died — stop acquiring; calibration
          // proceeds below with whatever samples were collected.
          std::cout << "Webcam source ended after " << std::max(image_count, 0)
                    << " accepted sample(s)." << std::endl;
          break;
        }
      }
#ifndef WEBCAM_ONLY
      else {
        frame_ok = (zed_cam.grab() == sl::ERROR_CODE::SUCCESS);
        if (frame_ok)
          zed_cam.retrieveImage(zed_image, sl::VIEW::LEFT_UNRECTIFIED_BGR);
      }
#endif

      if (frame_ok) {
        cv::resize(rgb, rgb_d, display_size);
        cv::resize(rgb, rgb_d_fill, display_size);

        applyCoverageIndicatorOverlay(rgb_d_fill, coverage_indicator,
                                      limits_indicator);
        applyPosIndicatorOverlay(rgb_d_fill, pos_indicator);

        // Detection. ChArUco marker detection is scale-sensitive, so it runs on
        // the full-resolution frame; the classic chessboard keeps its faster
        // downscaled detection. `pts`/`ids` are full-resolution corners.
        std::vector<cv::Point2f> pts;
        std::vector<int> ids;
        bool found = false;
        if (use_charuco) {
          BoardDetection det = detector.detect(rgb);
          found = det.found;
          pts = det.imagePoints;   // full-res
          ids = det.ids;
          // Draw a downscaled copy on the display image.
          std::vector<cv::Point2f> pts_disp = pts;
          scaleKP(pts_disp, cam_size, display_size);
          BoardDetection det_disp = det;
          det_disp.imagePoints = pts_disp;
          for (auto& mc : det_disp.markerCorners)
            scaleKP(mc, cam_size, display_size);
          detector.draw(rgb_d_fill, det_disp);
        } else {
          found = cv::findChessboardCorners(rgb_d, cv::Size(h_edges, v_edges),
                                            pts);  // display-res
          drawChessboardCorners(rgb_d_fill, cv::Size(h_edges, v_edges),
                                cv::Mat(pts), found);
        }

        display = rgb_d_fill;

        // Mirror the preview (selfie view) AFTER all scene-space overlays so
        // they flip with the image, and BEFORE the HUD text so it stays
        // readable. Detection/calibration always run on the unmirrored frame.
        if (args.mirror) cv::flip(display, display, 1);

        // ---- ChArUco live tuning HUD: detected corners vs the full grid.
        // Use it to tune distance / lighting / --dict / board params: aim to
        // maximize the count (full board is squares_x-1 x squares_y-1 corners).
        if (use_charuco) {
          const int total = detector.cornersX() * detector.cornersY();
          const int n = static_cast<int>(ids.size());
          const cv::Scalar hud_color =
              (n >= detector.minValidCorners()) ? info_color : warn_color;
          std::stringstream ss_ch;
          ss_ch << "ChArUco: " << n << "/" << total << " corners";
          cv::putText(display, ss_ch.str(), cv::Point(10, 24),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 4);
          cv::putText(display, ss_ch.str(), cv::Point(10, 24),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, hud_color, 2);
          if (n < detector.minValidCorners())
            cv::putText(display, "Move closer / improve lighting",
                        cv::Point(10, 48), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        warn_color, 2);
        }

        cv::Mat text_info = cv::Mat::ones(
            cv::Size(text_area_width, display.size[0]), display.type());

        if (acquisition_completed) {
          cv::putText(text_info,
                      "Acquisition completed!",
                      cv::Point(10, 50),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, info_color, 2);
          cv::putText(text_info,
                      "Wait for calibration to complete...",
                      cv::Point(10, 78), cv::FONT_HERSHEY_SIMPLEX,
                      0.6, info_color, 2);
          cv::putText(
              text_info,
              "Follow the console log for details.",
              cv::Point(10, 106), cv::FONT_HERSHEY_SIMPLEX,
              0.55, info_color, 2);
        } else {
          cv::putText(
              text_info,
              "Press 's' or spacebar to save the current frame.",
              cv::Point(10, 16), cv::FONT_HERSHEY_SIMPLEX,
              0.5, info_color, 1);
          cv::putText(
              text_info,
              "Move target to improve coverage and variability.",
              cv::Point(10, 34), cv::FONT_HERSHEY_SIMPLEX,
              0.45, warn_color, 1);

          // ----> Draw Status Info <---- //
          int v_pos = 60;
          int v_space = 28;
          int h_pos = 10;
          int h_space = 130;
          double font_scale = 0.6;

          auto draw_text_row = [&text_info, h_pos, h_space, font_scale,
                                 info_color, warn_color](
                                    const std::string& label, int v_pos,
                                    int min_val, int max_val, int req_i,
                                    float score) {
            cv::putText(text_info, label, cv::Point(h_pos, v_pos),
                        cv::FONT_HERSHEY_SIMPLEX, font_scale,
                        (score >= 1.0f ? info_color : warn_color), 1);
            cv::putText(text_info, std::to_string(min_val),
                        cv::Point(h_pos + h_space, v_pos),
                        cv::FONT_HERSHEY_SIMPLEX, font_scale,
                        (score >= 1.0f ? info_color : warn_color), 1);
            cv::putText(text_info, std::to_string(max_val),
                        cv::Point(h_pos + 2 * h_space, v_pos),
                        cv::FONT_HERSHEY_SIMPLEX, font_scale,
                        (score >= 1.0f ? info_color : warn_color), 1);
            cv::putText(text_info, std::to_string(max_val - min_val),
                        cv::Point(h_pos + 3 * h_space, v_pos),
                        cv::FONT_HERSHEY_SIMPLEX, font_scale,
                        (score >= 1.0f ? info_color : warn_color), 1);
            cv::putText(text_info, std::to_string(req_i),
                        cv::Point(h_pos + 4 * h_space, v_pos),
                        cv::FONT_HERSHEY_SIMPLEX, font_scale,
                        (score >= 1.0f ? info_color : warn_color), 1);
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << score * 100.0f << "%";
            cv::putText(text_info, ss.str(),
                        cv::Point(h_pos + 5 * h_space, v_pos),
                        cv::FONT_HERSHEY_SIMPLEX, font_scale,
                        (score >= 1.0f ? info_color : warn_color), 1);
          };

          cv::putText(text_info, "Sample Collection Status",
                      cv::Point(10, v_pos), cv::FONT_HERSHEY_SIMPLEX,
                      font_scale, info_color, 2);

          v_pos += v_space;
          cv::putText(text_info, "METRIC", cv::Point(h_pos, v_pos),
                      cv::FONT_HERSHEY_SIMPLEX, font_scale, info_color, 2);
          cv::putText(text_info, "MIN",
                      cv::Point(h_pos + h_space, v_pos),
                      cv::FONT_HERSHEY_SIMPLEX, font_scale, info_color, 2);
          cv::putText(text_info, "MAX",
                      cv::Point(h_pos + 2 * h_space, v_pos),
                      cv::FONT_HERSHEY_SIMPLEX, font_scale, info_color, 2);
          cv::putText(text_info, "RANGE",
                      cv::Point(h_pos + 3 * h_space, v_pos),
                      cv::FONT_HERSHEY_SIMPLEX, font_scale, info_color, 2);
          cv::putText(text_info, "REQ",
                      cv::Point(h_pos + 4 * h_space, v_pos),
                      cv::FONT_HERSHEY_SIMPLEX, font_scale, info_color, 2);
          cv::putText(text_info, "SCORE",
                      cv::Point(h_pos + 5 * h_space, v_pos),
                      cv::FONT_HERSHEY_SIMPLEX, font_scale, info_color, 2);

          v_pos += v_space;
          draw_text_row(
              "X [px]", v_pos,
              static_cast<int>(min_bx * cam_size.width),
              static_cast<int>(max_bx * cam_size.width),
              static_cast<int>(min_b_x_coverage * cam_size.width),
              pos_score_x);

          v_pos += v_space;
          draw_text_row(
              "Y [px]", v_pos,
              static_cast<int>(min_by * cam_size.height),
              static_cast<int>(max_by * cam_size.height),
              static_cast<int>(min_b_y_coverage * cam_size.height),
              pos_score_y);

          v_pos += v_space;
          draw_text_row(
              "Size [sq.px]", v_pos,
              static_cast<int>(min_size * cam_size.height * cam_size.width),
              static_cast<int>(max_size * cam_size.height * cam_size.width),
              static_cast<int>(min_area_range * cam_size.height *
                               cam_size.width),
              size_score);

          v_pos += v_space;
          draw_text_row("Skew [deg]", v_pos,
                        static_cast<int>(min_skew * 90.0f),
                        static_cast<int>(max_skew * 90.0f),
                        static_cast<int>(min_skew_range * 90.0f),
                        skew_score);

          v_pos += v_space;
          std::stringstream ss_img_count;
          ss_img_count << "Samples: " << std::max(image_count, 0)
                       << " [min. " << min_samples << ","
                       << " max. " << max_samples << "]";
          cv::putText(text_info, ss_img_count.str(),
                      cv::Point(10, v_pos), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                      (image_count > min_samples ? info_color : warn_color), 1);

          if (missing_target || low_target_variability || blurry_image) {
            cv::putText(
                text_info, "Frame not saved for calibration.",
                cv::Point(10, v_pos + 35),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, warn_color, 2);
          }
          if (missing_target) {
            cv::putText(
                text_info, " * Target not detected.",
                cv::Point(10, v_pos + 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, warn_color, 2);
          }
          if (low_target_variability) {
            cv::putText(
                text_info,
                " * Target too similar or too small.",
                cv::Point(10, v_pos + 85),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, warn_color, 2);
          }
          if (blurry_image) {
            cv::putText(
                text_info,
                " * Image too blurry. Hold the target still.",
                cv::Point(10, v_pos + 110),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, warn_color, 2);
          }

          // Auto-capture toggle button (click or key 'a').
          const cv::Scalar btn_fill = auto_btn.enabled
                                          ? cv::Scalar(40, 120, 40)
                                          : cv::Scalar(60, 60, 60);
          cv::rectangle(text_info, auto_btn_local, btn_fill, -1);
          cv::rectangle(text_info, auto_btn_local,
                        auto_btn.enabled ? info_color : cv::Scalar(160, 160, 160),
                        2);
          cv::putText(text_info,
                      auto_btn.enabled ? "AUTO CAPTURE: ON  [a]"
                                       : "AUTO CAPTURE: OFF [a]",
                      cv::Point(auto_btn_local.x + 14, auto_btn_local.y + 25),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6,
                      cv::Scalar(255, 255, 255), 2);
        }

        cv::hconcat(display, text_info, rendering_image);
        cv::imshow(window_name, rendering_image);
        key = cv::waitKey(10);

        // Window closed via the title-bar X: shut down like 'q'. The GTK
        // backend does not implement WND_PROP_VISIBLE (always -1), but any
        // property query returns -1 once the window is destroyed — so probe
        // WND_PROP_AUTOSIZE (>= 0 while the window exists) and only treat -1
        // as "closed" after the window has been seen alive once.
        const bool window_exists =
            cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0;
        if (window_exists) {
          window_was_visible = true;
        } else if (window_was_visible) {
          g_stop_requested = true;
          continue;
        }

        if (acquisition_completed) {
          std::cout << " *** Starting the calibration process ***" << std::endl;
          break;
        }

        if (key == 'a' || key == 'A') auto_btn.enabled = !auto_btn.enabled;

        // Manual capture on keypress; auto capture attempts a (throttled)
        // testSample whenever a board is detected — the checker only accepts
        // frames that improve sample diversity, so holding the board still
        // does not flood the database with duplicates.
        const bool manual_capture = (key == 's' || key == 'S' || key == ' ');
        const double now_s =
            static_cast<double>(cv::getTickCount()) / cv::getTickFrequency();
        const bool auto_attempt = auto_btn.enabled && !manual_capture &&
                                  found && now_s >= next_auto_attempt;

        if (manual_capture || auto_attempt) {
          bool sample_accepted = false;
          if (manual_capture) {
            std::cout << "*** New acquisition triggered ***" << std::endl;
            missing_target = !found;
            blurry_image = false;
          }

          if (found) {
            // `pts` is display-res for a chessboard, full-res for ChArUco.
            // Build a full-res set (for testSample) and a display-res set (for
            // the coverage polygon overlay).
            std::vector<cv::Point2f> full_pts = pts;
            std::vector<cv::Point2f> disp_pts = pts;
            if (use_charuco) {
              scaleKP(disp_pts, cam_size, display_size);
            } else {
              scaleKP(full_pts, display_size, cam_size);
            }

            double sharpness = computeSharpness(rgb);
            const bool too_blurry = sharpness < min_sharpness;
            if (manual_capture) blurry_image = too_blurry;
            if (too_blurry) {
              if (manual_capture)
                std::cerr << "  ! Image too blurry (sharpness=" << std::fixed
                          << std::setprecision(1) << sharpness
                          << ", min=" << min_sharpness
                          << "). Hold still and retry." << std::endl;
            } else if (checker.testSample(full_pts, cam_size, ids)) {
              sample_accepted = true;
              low_target_variability = false;

              if (image_count < 0) image_count = 0;
              std::string img_path =
                  image_folder + "image_" + std::to_string(image_count) + ".png";
              cv::imwrite(img_path, rgb);
              std::cout << " * Image saved: '" << img_path << "'" << std::endl;
              image_count++;

              if (checker.evaluateSampleCollectionStatus(
                      size_score, skew_score, pos_score_x, pos_score_y,
                      min_size, max_size, min_skew, max_skew, min_bx, max_bx,
                      min_by, max_by)) {
                std::cout << ">>> Sample collection status: COMPLETE <<<"
                          << std::endl
                          << std::endl;
                acquisition_completed = true;
              }

              float norm_x = checker.getLastDetectedBoardParams().avg_pos.x;
              float norm_y = checker.getLastDetectedBoardParams().avg_pos.y;
              float norm_size = checker.getLastDetectedBoardParams().size;
              addNewCheckerboardPosition(coverage_indicator, pos_indicator,
                                         limits_indicator, norm_x, norm_y,
                                         norm_size, min_bx, max_bx, min_by,
                                         max_by, (image_count >= 2));
              // 4 outside corners (TL,TR,BR,BL) in display coordinates.
              std::vector<cv::Point2f> poly;
              if (use_charuco) {
                poly = detector.extremeCorners(disp_pts, ids);
              } else {
                poly = {disp_pts[0], disp_pts[h_edges - 1],
                        disp_pts[disp_pts.size() - 1],
                        disp_pts[disp_pts.size() - h_edges]};
              }
              if (poly.size() == 4)
                addNewCheckerboardPoly(coverage_indicator, poly);
            } else if (manual_capture) {
              std::cout << "  ! Checkerboard detected, but sample not valid. "
                           "Please try again with a new position/orientation."
                        << std::endl;
              low_target_variability = true;
            }
          } else {
            std::cerr << "  ! Checkerboard not detected in the image."
                      << std::endl;
          }

          // Throttle auto attempts: a longer pause after an accepted sample
          // gives the operator time to move the board to a new pose.
          if (auto_attempt)
            next_auto_attempt = now_s + (sample_accepted ? 1.2 : 0.5);
        }
      }
    }

    if (args.use_webcam) {
      cap.release();
    }
#ifndef WEBCAM_ONLY
    else {
      zed_cam.close();
    }
#endif
  }

  int err = calibrate(image_count, image_folder, calib, board_cfg,
                      serial_number, is_4k_camera, can_use_calib_prior,
                      max_repr_error, verbose);

  if (err == EXIT_SUCCESS)
    std::cout << std::endl
              << " +++++ Calibration successful +++++" << std::endl;
  else
    std::cout << std::endl << " ----- Calibration failed -----" << std::endl;

  return EXIT_SUCCESS;
}

static int top_left_count = 0;
static int top_right_count = 0;
static int bottom_left_count = 0;
static int bottom_right_count = 0;

void addNewCheckerboardPosition(cv::Mat& coverage_indicator,
                                cv::Mat& pos_indicator,
                                cv::Mat& limits_indicator, float norm_x,
                                float norm_y, float norm_size, float min_x,
                                float max_x, float min_y, float max_y,
                                bool draw_rect) {
  int x = static_cast<int>(norm_x * pos_indicator.cols);
  int y = static_cast<int>(norm_y * pos_indicator.rows);
  int size = static_cast<int>(norm_size * 30.0f);
  cv::circle(pos_indicator, cv::Point(x, y), size, cv::Scalar(255, 255, 255), -1);

  int min_x_px = static_cast<int>(min_x * pos_indicator.cols);
  int max_x_px = static_cast<int>(max_x * pos_indicator.cols);
  int min_y_px = static_cast<int>(min_y * pos_indicator.rows);
  int max_y_px = static_cast<int>(max_y * pos_indicator.rows);

  limits_indicator.setTo(cv::Scalar(0, 0, 0));

  if (draw_rect) {
    int col_val = 50;
    cv::rectangle(limits_indicator, cv::Point(0, 0),
                  cv::Point(min_x_px, limits_indicator.rows - 1),
                  cv::Scalar(col_val, col_val, col_val), -1);
    cv::rectangle(limits_indicator, cv::Point(0, 0),
                  cv::Point(limits_indicator.cols - 1, min_y_px),
                  cv::Scalar(col_val, col_val, col_val), -1);
    cv::rectangle(limits_indicator,
                  cv::Point(0, max_y_px),
                  cv::Point(limits_indicator.cols - 1, limits_indicator.rows - 1),
                  cv::Scalar(col_val, col_val, col_val), -1);
    cv::rectangle(limits_indicator,
                  cv::Point(max_x_px, 0),
                  cv::Point(limits_indicator.cols - 1, limits_indicator.rows - 1),
                  cv::Scalar(col_val, col_val, col_val), -1);
  }
  cv::line(limits_indicator, cv::Point(min_x * limits_indicator.cols, 0),
           cv::Point(min_x * limits_indicator.cols, limits_indicator.rows - 1),
           cv::Scalar(255, 255, 255), 2);
  cv::line(limits_indicator, cv::Point(max_x * limits_indicator.cols, 0),
           cv::Point(max_x * limits_indicator.cols, limits_indicator.rows - 1),
           cv::Scalar(255, 255, 255), 2);
  cv::line(limits_indicator, cv::Point(0, min_y * limits_indicator.rows),
           cv::Point(limits_indicator.cols - 1, min_y * limits_indicator.rows),
           cv::Scalar(255, 255, 255), 2);
  cv::line(limits_indicator, cv::Point(0, max_y * limits_indicator.rows),
           cv::Point(limits_indicator.cols - 1, max_y * limits_indicator.rows),
           cv::Scalar(255, 255, 255), 2);

  if (norm_x < 0.5f && norm_y < 0.5f) {
    top_left_count++;
  } else if (norm_x >= 0.5f && norm_y < 0.5f) {
    top_right_count++;
  } else if (norm_x < 0.5f && norm_y >= 0.5f) {
    bottom_left_count++;
  } else {
    bottom_right_count++;
  }

  if (top_left_count >= min_samples / 4) {
    cv::rectangle(coverage_indicator, cv::Point(0, 0),
                  cv::Point(coverage_indicator.cols / 2, coverage_indicator.rows / 2),
                  cv::Scalar(255), -1);
  }
  if (top_right_count >= min_samples / 4) {
    cv::rectangle(coverage_indicator,
                  cv::Point(coverage_indicator.cols / 2, 0),
                  cv::Point(coverage_indicator.cols, coverage_indicator.rows / 2),
                  cv::Scalar(255), -1);
  }
  if (bottom_left_count >= min_samples / 4) {
    cv::rectangle(coverage_indicator,
                  cv::Point(0, coverage_indicator.rows / 2),
                  cv::Point(coverage_indicator.cols / 2, coverage_indicator.rows),
                  cv::Scalar(255), -1);
  }
  if (bottom_right_count >= min_samples / 4) {
    cv::rectangle(coverage_indicator,
                  cv::Point(coverage_indicator.cols / 2, coverage_indicator.rows / 2),
                  cv::Point(coverage_indicator.cols, coverage_indicator.rows),
                  cv::Scalar(255), -1);
  }
}

// `pts` holds the 4 outside corners in TL, TR, BR, BL order (display coords).
void addNewCheckerboardPoly(cv::Mat& coverage_indicator,
                            const std::vector<cv::Point2f>& pts) {
  cv::Point tl = pts[0];
  cv::Point tr = pts[1];
  cv::Point br = pts[2];
  cv::Point bl = pts[3];

  std::vector<cv::Point> poly_pts = {tl, tr, br, bl};
  cv::Mat mask = cv::Mat::zeros(coverage_indicator.size(), CV_8UC1);
  cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{poly_pts},
               cv::Scalar(255 / 5, 255 / 5, 255 / 5));
  coverage_indicator = coverage_indicator + mask;
}

void applyCoverageIndicatorOverlay(cv::Mat& image,
                                   const cv::Mat& coverage_indicator,
                                   const cv::Mat& limits_indicator) {
  std::vector<cv::Mat> channels;
  cv::split(image, channels);
  channels[0] = channels[0] - coverage_indicator;
  channels[2] = channels[2] - coverage_indicator;
  channels[0] = channels[0] - limits_indicator;
  channels[1] = channels[1] - limits_indicator;
  cv::merge(channels, image);
}

void applyPosIndicatorOverlay(cv::Mat& image, const cv::Mat& pos_indicator) {
  std::vector<cv::Mat> channels;
  cv::split(image, channels);
  channels[2] = channels[2] - pos_indicator;
  channels[1] = channels[1] - pos_indicator;
  cv::merge(channels, image);
}
