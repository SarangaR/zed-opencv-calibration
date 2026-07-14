#include <atomic>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>

#include <opencv2/opencv.hpp>
#ifndef WEBCAM_ONLY
#include <sl/Camera.hpp>
#include <sl/CameraOne.hpp>
#endif

#include "board_detector.hpp"
#include "http_streamer.hpp"
#include "web_common.hpp"

namespace mono_check_mode {

// *********************************************************************************
// CHANGE THIS PARAMS USING THE COMMAND LINE OPTIONS
// Learn more:
// * https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html

int h_edges = 9;           // number of horizontal inner edges
int v_edges = 6;           // number of vertical inner edges
float square_size = 25.4f; // mm (chessboard default; ChArUco defaults to 15)
bool square_size_set = false;  // true when --square_size was passed explicitly

// ChArUco board (enabled with --charuco). Defaults match the AndyMark board.
bool use_charuco = false;
int squares_x = 15;
int squares_y = 11;
float marker_size = 11.0f;
std::string dict_name = "DICT_4X4_250";
bool charuco_legacy = false;

// Default parameters are good for this checkerboard:
// https://github.com/opencv/opencv/blob/4.x/doc/pattern.png/
// *********************************************************************************

bool verbose = false;
int sdk_verbose = 0;

cv::Size display_size(720, 404);  // 16:9 default; width follows source aspect
constexpr int text_area_height = 120;

// ---------------------------------------------------------------------------------
// Calibration data retrieved from the ZED SDK after camera open
// ---------------------------------------------------------------------------------
struct MonoCalibData {
    cv::Mat K;
    cv::Mat D;
    bool is_fisheye = false;

    // Load intrinsics from a YAML produced by zed_mono_calibration
    // (keys: K + D for RadTan, or K + D_FE for fisheye). Used in webcam test
    // mode where no ZED SDK calibration exists.
    bool loadFromYml(const std::string& path) {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;
        fs["K"] >> K;
        if (!fs["D"].empty()) {
            fs["D"] >> D;
            is_fisheye = false;
        } else if (!fs["D_FE"].empty()) {
            fs["D_FE"] >> D;
            is_fisheye = true;
        } else {
            return false;
        }
        return !K.empty() && !D.empty();
    }

#ifndef WEBCAM_ONLY
    void setFrom(const sl::CameraParameters& p) {
        K = cv::Mat::eye(3, 3, CV_64FC1);
        K.at<double>(0, 0) = p.fx;
        K.at<double>(1, 1) = p.fy;
        K.at<double>(0, 2) = p.cx;
        K.at<double>(1, 2) = p.cy;

        // Fisheye: p1=p2=0 but k3,k4≠0 (indices 2,3 vs 4,5 in disto array)
        is_fisheye = (p.disto[2] == 0. && p.disto[3] == 0. &&
                      p.disto[4] != 0. && p.disto[5] != 0.);

        if (is_fisheye) {
            D = cv::Mat::zeros(1, 4, CV_64FC1);
            D.at<double>(0) = p.disto[0];
            D.at<double>(1) = p.disto[1];
            D.at<double>(2) = p.disto[4];
            D.at<double>(3) = p.disto[5];
        } else {
            D = cv::Mat::zeros(1, 8, CV_64FC1);
            for (int i = 0; i < 8; i++) D.at<double>(i) = p.disto[i];
        }
    }
#endif  // WEBCAM_ONLY
};

// ---------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------
void scaleKP(std::vector<cv::Point2f>& pts, cv::Size in, cv::Size out) {
    float rx = out.width / static_cast<float>(in.width);
    float ry = out.height / static_cast<float>(in.height);
    for (auto& p : pts) {
        p.x *= rx;
        p.y *= ry;
    }
}

// Solve PnP with fisheye undistortion workaround (cv::solvePnP does not support fisheye)
// Degenerate point sets (e.g. near-collinear partial ChArUco views) make
// solvePnP throw rather than return false — treat that as "no pose".
bool computePose(const std::vector<cv::Point3f>& obj_pts,
                 const std::vector<cv::Point2f>& img_pts,
                 const cv::Mat& K, const cv::Mat& D,
                 bool is_fisheye,
                 cv::Mat& rvec, cv::Mat& tvec) {
    try {
        if (is_fisheye) {
            std::vector<cv::Point2f> undist;
            cv::fisheye::undistortPoints(img_pts, undist, K, D, cv::Mat(), K);
            cv::Mat zero_D = cv::Mat::zeros(1, 4, CV_64F);
            return cv::solvePnP(obj_pts, undist, K, zero_D, rvec, tvec);
        }
        return cv::solvePnP(obj_pts, img_pts, K, D, rvec, tvec);
    } catch (const cv::Exception&) {
        return false;
    }
}

double computeReprojError(const std::vector<cv::Point3f>& obj_pts,
                          const std::vector<cv::Point2f>& img_pts,
                          const cv::Mat& rvec, const cv::Mat& tvec,
                          const cv::Mat& K, const cv::Mat& D,
                          bool is_fisheye) {
    std::vector<cv::Point2f> proj;
    if (is_fisheye) {
        cv::fisheye::projectPoints(obj_pts, proj, rvec, tvec, K, D);
    } else {
        cv::projectPoints(obj_pts, rvec, tvec, K, D, proj);
    }

    double sq = 0.0;
    for (int i = 0; i < static_cast<int>(img_pts.size()); i++) {
        double dx = proj[i].x - img_pts[i].x;
        double dy = proj[i].y - img_pts[i].y;
        sq += dx * dx + dy * dy;
    }
    return std::sqrt(sq / img_pts.size());
}

void drawErrorText(cv::Mat& img, const std::string& label, double err, cv::Point pos) {
    std::stringstream ss;
    cv::Scalar color;
    if (err < 0.0) {
        ss << label << ": N/A";
        color = cv::Scalar(180, 180, 180);
    } else {
        ss << label << ": " << std::fixed << std::setprecision(3) << err << " px";
        if (err < 0.5)
            color = cv::Scalar(50, 210, 50);
        else if (err < 1.0)
            color = cv::Scalar(0, 165, 255);
        else
            color = cv::Scalar(0, 50, 250);
    }
    // Dark shadow for readability on any background
    cv::putText(img, ss.str(), pos + cv::Point(1, 1), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(0, 0, 0), 3);
    cv::putText(img, ss.str(), pos, cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
}

// ---------------------------------------------------------------------------------
// Argument parsing
// ---------------------------------------------------------------------------------
struct Args {
    std::string app_name;
    std::string svo_path;
    bool is_radtan_lens = true;
    int camera_id = -1;
    int camera_sn = -1;
    std::string calib_opencv_file;  // optional: passed to InitParametersOne
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
            } else if (arg == "--calib_opencv" && i + 1 < argc) {
                calib_opencv_file = argv[++i];
            } else if (arg == "--help" || arg == "-h") {
                std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
                std::cout << "  --calib_opencv <file>  Path to an OpenCV YAML calibration file" << std::endl;
                std::cout << "                         (if omitted, the camera's internal calibration is used)" << std::endl;
                std::cout << "  --h_edges <value>      Number of horizontal inner edges of the checkerboard" << std::endl;
                std::cout << "  --v_edges <value>      Number of vertical inner edges of the checkerboard" << std::endl;
                std::cout << "  --square_size <value>  Square size in mm (default 25.4 chessboard / 15 ChArUco)" << std::endl;
                std::cout << "  --charuco              Use a ChArUco board instead of a plain chessboard" << std::endl;
                std::cout << "  --squares_x <value>    ChArUco: squares along X (default 15)" << std::endl;
                std::cout << "  --squares_y <value>    ChArUco: squares along Y (default 11)" << std::endl;
                std::cout << "  --marker_size <value>  ChArUco: marker size in mm (default 11)" << std::endl;
                std::cout << "  --dict <name>          ChArUco: ArUco dictionary (default DICT_4X4_250)" << std::endl;
                std::cout << "  --charuco_legacy       ChArUco: board uses the pre-4.7 (legacy) layout" << std::endl;
                std::cout << "  --svo <file>           Path to the SVO file" << std::endl;
                std::cout << "  --webcam <id|path>     Test mode: plain webcam (device id) or video file instead of a ZED (requires --calib_opencv)" << std::endl;
                std::cout << "  --mirror               Mirror the live preview horizontally (selfie view; display only)" << std::endl;
                std::cout << "  --fisheye              Use fisheye lens model" << std::endl;
                std::cout << "  --id <id>              Camera ID of the ZED X One" << std::endl;
                std::cout << "  --sn <sn>              Serial number of the ZED X One" << std::endl;
                std::cout << "  --help, -h             Show this help message" << std::endl << std::endl;
                std::cout << "Examples:" << std::endl;
                std::cout << std::endl << "* ZED X One with internal calibration:" << std::endl;
                std::cout << "  " << argv[0] << std::endl;
                std::cout << std::endl << "* ZED X One with an OpenCV calibration file:" << std::endl;
                std::cout << "  " << argv[0] << " --calib_opencv zed_calibration_12345678.yml" << std::endl;
                std::cout << std::endl << "* ZED X One selected by serial number:" << std::endl;
                std::cout << "  " << argv[0] << " --sn 12345678" << std::endl;
                std::cout << std::endl << "* ZED X One with fisheye lens and custom checkerboard:" << std::endl;
                std::cout << "  " << argv[0]
                          << " --fisheye --h_edges 12 --v_edges 9 --square_size 30.0"
                          << std::endl;
                std::cout << std::endl;
                exit(0);
            }
        }
    }
};

// ---------------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------------
int run(int argc, char* argv[], HttpStreamer& streamer) {
    Args args;
    args.parse(argc, argv);

    // ChArUco defaults match the AndyMark board (15 mm squares); 25.4 mm is
    // the chessboard default. Mismatched square size scales metric outputs.
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

    std::cout << "*** Monocular Calibration Checker (ZED X One) ***" << std::endl << std::endl;
    if (use_charuco) {
        std::cout << " * Expected ChArUco board features:" << std::endl;
        std::cout << "   - Squares (X x Y):\t\t" << squares_x << " x " << squares_y << std::endl;
        std::cout << "   - Square size:\t\t" << square_size << " mm" << std::endl;
        std::cout << "   - Marker size:\t\t" << marker_size << " mm" << std::endl;
        std::cout << "   - ArUco dictionary:\t\t" << dict_name << std::endl;
    } else {
        std::cout << " * Expected checkerboard features:" << std::endl;
        std::cout << "   - Inner horizontal edges:\t" << h_edges << std::endl;
        std::cout << "   - Inner vertical edges:\t" << v_edges << std::endl;
        std::cout << "   - Square size:\t\t" << square_size << " mm" << std::endl;
    }
    std::cout << "Change these parameters using the command line options if needed. Use '-h' for help." << std::endl << std::endl;

    // ---- Frame source + calibration initialization ----
    // Default: ZED X One (SDK). Test mode (--webcam): plain cv::VideoCapture +
    // intrinsics loaded straight from a --calib_opencv YAML file.
    cv::VideoCapture cap;
    cv::Size full_size;
    cv::Mat rgb;
    MonoCalibData calib;
#ifndef WEBCAM_ONLY
    sl::CameraOne zed_cam;
    sl::Mat zed_img;
#endif

    if (args.use_webcam) {
        if (args.calib_opencv_file.empty()) {
            std::cerr << "--webcam requires --calib_opencv <file>: without the "
                         "ZED SDK there is no internal calibration to check."
                      << std::endl;
            return EXIT_FAILURE;
        }
        if (!calib.loadFromYml(args.calib_opencv_file)) {
            std::cerr << "Cannot load calibration from '"
                      << args.calib_opencv_file << "'." << std::endl;
            return EXIT_FAILURE;
        }
        const bool is_id =
            !args.webcam_source.empty() &&
            args.webcam_source.find_first_not_of("0123456789") ==
                std::string::npos;
        const bool opened = is_id ? cap.open(std::stoi(args.webcam_source))
                                  : cap.open(args.webcam_source);
        if (!opened || !cap.isOpened() || !cap.read(rgb) || rgb.empty()) {
            std::cerr << "Error opening webcam source '" << args.webcam_source
                      << "'." << std::endl;
            return EXIT_FAILURE;
        }
        full_size = rgb.size();
        std::cout << " * Webcam test mode. Source: " << args.webcam_source
                  << " (" << full_size.width << " x " << full_size.height
                  << ")" << std::endl;
    } else {
#ifdef WEBCAM_ONLY
        std::cerr << "This binary was built without the ZED SDK (WEBCAM_ONLY). "
                     "Use --webcam <id-or-path> --calib_opencv <file>."
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

        if (!args.calib_opencv_file.empty())
            init_params.optional_opencv_calibration_file = args.calib_opencv_file.c_str();

        auto status = zed_cam.open(init_params);
        if (status > sl::ERROR_CODE::SUCCESS &&
            status != sl::ERROR_CODE::INVALID_CALIBRATION_FILE) {
            std::cerr << "Error opening ZED X One camera: " << sl::toString(status) << std::endl;
            return EXIT_FAILURE;
        }

        auto zed_info = zed_cam.getCameraInformation();
        sl::Resolution cam_res = zed_info.camera_configuration.resolution;
        full_size = cv::Size(static_cast<int>(cam_res.width),
                             static_cast<int>(cam_res.height));

        std::cout << " * Camera Model: " << sl::toString(zed_info.camera_model) << std::endl;
        std::cout << " * Camera S/N: " << zed_info.serial_number << std::endl;
        std::cout << " * Camera Resolution: " << full_size.width << " x " << full_size.height << std::endl;

        // Populate calibration from the SDK (reflects the active calibration source)
        calib.setFrom(zed_info.camera_configuration.calibration_parameters_raw);

        zed_img.alloc(cam_res, sl::MAT_TYPE::U8_C3, sl::MEM::CPU);
        rgb = cv::Mat(full_size.height, full_size.width, CV_8UC3,
                      zed_img.getPtr<sl::uchar1>());
#endif
    }

    // Match the preview aspect to the source so the image is not stretched
    // (e.g. 4:3 webcams vs the 16:9 default).
    display_size.width =
        display_size.height * full_size.width / full_size.height;

    const std::string calib_source = args.calib_opencv_file.empty()
                                         ? "Internal ZED calibration"
                                         : "OpenCV file: " + args.calib_opencv_file;
    std::cout << " * Calibration source: " << calib_source << std::endl;
    std::cout << " * Distortion model: "
              << (calib.is_fisheye ? "Fisheye" : "Radial-Tangential") << std::endl << std::endl;

    // 3D points of the checkerboard pattern
    std::vector<cv::Point3f> obj_pts;
    for (int i = 0; i < v_edges; i++)
        for (int j = 0; j < h_edges; j++)
            obj_pts.push_back(cv::Point3f(square_size * j, square_size * i, 0.f));

    // Sub-pixel refinement window: larger for higher resolution
    const int subpix_win = (full_size.width >= 3000) ? 15 : 11;

    // Keep last valid error value so the display does not flicker when the
    // target temporarily disappears
    double disp_err = -1.0;

    std::cout << "Point the camera at the checkerboard. Stop from the web UI." << std::endl;

    while (1) {
        if (g_stop_requested) break;

        if (args.use_webcam) {
            if (!cap.read(rgb) || rgb.empty()) {
                std::cout << "Webcam source ended." << std::endl;
                break;
            }
        }
#ifndef WEBCAM_ONLY
        else {
            if (zed_cam.grab() != sl::ERROR_CODE::SUCCESS) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            zed_cam.retrieveImage(zed_img, sl::VIEW::LEFT_UNRECTIFIED_BGR);
        }
#endif

        cv::Mat rgb_d;
        cv::resize(rgb, rgb_d, display_size);

        double err = -1.0;
        bool found = false;

        if (use_charuco) {
            // ChArUco: full-resolution detection (marker detection is scale-sensitive).
            BoardDetection det = detector.detect(rgb);
            found = det.found;
            if (found) {
                cv::Mat rvec, tvec;
                if (computePose(det.objectPoints, det.imagePoints, calib.K,
                                calib.D, calib.is_fisheye, rvec, tvec)) {
                    err = computeReprojError(det.objectPoints, det.imagePoints,
                                             rvec, tvec, calib.K, calib.D,
                                             calib.is_fisheye);
                }
            }
            // Draw a downscaled copy on the display image: corner dots when
            // the board is valid, raw ArUco tag outlines always (shows which
            // markers the camera picks up even when the board detection fails).
            std::vector<cv::Point2f> disp = det.imagePoints;
            scaleKP(disp, full_size, display_size);
            BoardDetection det_disp = det;
            det_disp.imagePoints = disp;
            for (auto& mc : det_disp.markerCorners)
                scaleKP(mc, full_size, display_size);
            detector.draw(rgb_d, det_disp);
            // Mirror the preview (selfie view) AFTER scene-space overlays,
            // BEFORE the HUD text so it stays readable.
            if (args.mirror) cv::flip(rgb_d, rgb_d, 1);
            // ChArUco tuning HUD: detected corners vs full grid.
            {
                const int total = detector.cornersX() * detector.cornersY();
                std::stringstream ss_ch;
                ss_ch << "ChArUco: " << det.ids.size() << "/" << total << " corners";
                cv::putText(rgb_d, ss_ch.str(), cv::Point(10, 70),
                            cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 3);
                cv::putText(rgb_d, ss_ch.str(), cv::Point(10, 70),
                            cv::FONT_HERSHEY_SIMPLEX, 0.55,
                            cv::Scalar(50, 210, 50), 1);
            }
        } else {
            // ---- Chessboard detection on downscaled image (fast) ----
            std::vector<cv::Point2f> pts_d;
            found = cv::findChessboardCorners(rgb_d, cv::Size(h_edges, v_edges), pts_d);
            if (found) {
                // Scale corners to full resolution and refine sub-pixel
                std::vector<cv::Point2f> pts_f = pts_d;
                scaleKP(pts_f, display_size, full_size);

                cv::Mat gray;
                cv::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY);
                cv::cornerSubPix(gray, pts_f, cv::Size(subpix_win, subpix_win),
                                 cv::Size(-1, -1),
                                 cv::TermCriteria(cv::TermCriteria::EPS |
                                                      cv::TermCriteria::MAX_ITER,
                                                  30, 0.001));

                cv::Mat rvec, tvec;
                if (computePose(obj_pts, pts_f, calib.K, calib.D, calib.is_fisheye, rvec, tvec)) {
                    err = computeReprojError(obj_pts, pts_f, rvec, tvec,
                                            calib.K, calib.D, calib.is_fisheye);
                }

                cv::drawChessboardCorners(rgb_d, cv::Size(h_edges, v_edges),
                                          cv::Mat(pts_d), found);
            }
            if (args.mirror) cv::flip(rgb_d, rgb_d, 1);
        }

        // Update persisted display value; reset when target disappears
        disp_err = found ? err : -1.0;

        // ---- Draw error value directly on the image ----
        drawErrorText(rgb_d, "Reproj.", disp_err, cv::Point(10, 35));

        // ---- Compose final display ----
        cv::Mat text_area_mat =
            cv::Mat::ones(cv::Size(display_size.width, text_area_height), rgb_d.type());
        cv::Mat display;
        cv::vconcat(rgb_d, text_area_mat, display);

        const int ty_base = display_size.height + 35;

        // Color legend
        cv::putText(display, "< 0.5 px: good", cv::Point(10, ty_base),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50, 210, 50), 1);
        cv::putText(display, "0.5-1.0 px: acceptable", cv::Point(190, ty_base),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 165, 255), 1);
        cv::putText(display, "> 1.0 px: poor", cv::Point(440, ty_base),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 50, 250), 1);

        // Calibration source label
        cv::putText(display, "Calib: " + calib_source,
                    cv::Point(10, display_size.height + text_area_height - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(150, 150, 150), 1);

        streamer.publish(display);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (args.use_webcam) {
        cap.release();
    }
#ifndef WEBCAM_ONLY
    else {
        zed_cam.close();
    }
#endif
    return EXIT_SUCCESS;
}

}  // namespace mono_check_mode

int run_mono_check(int argc, char** argv, HttpStreamer& streamer) {
    return mono_check_mode::run(argc, argv, streamer);
}
