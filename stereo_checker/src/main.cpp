#include <atomic>
#include <csignal>
#include <iomanip>
#include <sstream>
#include <unordered_map>

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <sl/CameraOne.hpp>

#include "board_detector.hpp"

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

const cv::Size display_size(720, 404);
constexpr int text_area_height = 140;
const bool image_stack_horizontal = true;

// ---------------------------------------------------------------------------------
// Calibration data retrieved from the ZED SDK after camera open
// ---------------------------------------------------------------------------------
struct StereoCalibData {
    cv::Mat K_left, D_left;
    cv::Mat K_right, D_right;
    cv::Mat R;  // rotation matrix (left -> right)
    cv::Mat T;  // translation vector
    bool is_fisheye = false;

    void setFrom(const sl::CalibrationParameters& p) {
        // Left intrinsics
        const auto& lc = p.left_cam;
        K_left = cv::Mat::eye(3, 3, CV_64FC1);
        K_left.at<double>(0, 0) = lc.fx;
        K_left.at<double>(1, 1) = lc.fy;
        K_left.at<double>(0, 2) = lc.cx;
        K_left.at<double>(1, 2) = lc.cy;

        // Fisheye: p1=p2=0 but k3,k4≠0 (indices 2,3 vs 4,5 in disto array)
        is_fisheye = (lc.disto[2] == 0. && lc.disto[3] == 0. &&
                      lc.disto[4] != 0. && lc.disto[5] != 0.);

        if (is_fisheye) {
            D_left = cv::Mat::zeros(1, 4, CV_64FC1);
            D_left.at<double>(0) = lc.disto[0];
            D_left.at<double>(1) = lc.disto[1];
            D_left.at<double>(2) = lc.disto[4];
            D_left.at<double>(3) = lc.disto[5];
        } else {
            D_left = cv::Mat::zeros(1, 8, CV_64FC1);
            for (int i = 0; i < 8; i++) D_left.at<double>(i) = lc.disto[i];
        }

        // Right intrinsics
        const auto& rc = p.right_cam;
        K_right = cv::Mat::eye(3, 3, CV_64FC1);
        K_right.at<double>(0, 0) = rc.fx;
        K_right.at<double>(1, 1) = rc.fy;
        K_right.at<double>(0, 2) = rc.cx;
        K_right.at<double>(1, 2) = rc.cy;

        if (is_fisheye) {
            D_right = cv::Mat::zeros(1, 4, CV_64FC1);
            D_right.at<double>(0) = rc.disto[0];
            D_right.at<double>(1) = rc.disto[1];
            D_right.at<double>(2) = rc.disto[4];
            D_right.at<double>(3) = rc.disto[5];
        } else {
            D_right = cv::Mat::zeros(1, 8, CV_64FC1);
            for (int i = 0; i < 8; i++) D_right.at<double>(i) = rc.disto[i];
        }

        // Stereo extrinsics
        auto translation = p.stereo_transform.getTranslation();
        T = cv::Mat::zeros(3, 1, CV_64FC1);
        T.at<double>(0) = translation.x * -1;  // ZED SDK stores positive Tx
        T.at<double>(1) = translation.y;
        T.at<double>(2) = translation.z;

        auto rot = p.stereo_transform.getRotationVector();
        cv::Mat Rv = cv::Mat::zeros(3, 1, CV_64FC1);
        Rv.at<double>(0) = rot.x;
        Rv.at<double>(1) = rot.y;
        Rv.at<double>(2) = rot.z;
        cv::Rodrigues(Rv, R);
    }
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
    bool is_zed_x_one_virtual_stereo = false;
    int left_camera_id = -1;
    int right_camera_id = -1;
    int left_camera_sn = -1;
    int right_camera_sn = -1;
    std::string calib_opencv_file;  // optional: passed to InitParameters
    bool mirror = false;            // horizontally mirror the live preview only

    void parse(int argc, char* argv[]) {
        app_name = argv[0];
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "--svo" && i + 1 < argc) {
                svo_path = argv[++i];
            } else if (arg == "--mirror") {
                mirror = true;
            } else if (arg == "--fisheye") {
                is_radtan_lens = false;
            } else if (arg == "--virtual") {
                is_zed_x_one_virtual_stereo = true;
            } else if (arg == "--left_id" && i + 1 < argc) {
                left_camera_id = std::stoi(argv[++i]);
            } else if (arg == "--right_id" && i + 1 < argc) {
                right_camera_id = std::stoi(argv[++i]);
            } else if (arg == "--left_sn" && i + 1 < argc) {
                left_camera_sn = std::stoi(argv[++i]);
            } else if (arg == "--right_sn" && i + 1 < argc) {
                right_camera_sn = std::stoi(argv[++i]);
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
                std::cout << "  --mirror               Mirror the live preview horizontally (selfie view; display only)" << std::endl;
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
                std::cout << "  --fisheye              Use fisheye lens model" << std::endl;
                std::cout << "  --virtual              Use ZED X One cameras as a virtual stereo pair" << std::endl;
                std::cout << "  --left_id <id>         Id of the left camera if using virtual stereo" << std::endl;
                std::cout << "  --right_id <id>        Id of the right camera if using virtual stereo" << std::endl;
                std::cout << "  --left_sn <sn>         S/N of the left camera if using virtual stereo" << std::endl;
                std::cout << "  --right_sn <sn>        S/N of the right camera if using virtual stereo" << std::endl;
                std::cout << "  --help, -h             Show this help message" << std::endl << std::endl;
                std::cout << "Examples:" << std::endl;
                std::cout << std::endl << "* ZED Stereo Camera with internal calibration:" << std::endl;
                std::cout << "  " << argv[0] << std::endl;
                std::cout << std::endl << "* ZED Stereo Camera with an OpenCV calibration file:" << std::endl;
                std::cout << "  " << argv[0] << " --calib_opencv zed_calibration_12345678.yml" << std::endl;
                std::cout << std::endl << "* Virtual Stereo Camera using serial numbers:" << std::endl;
                std::cout << "  " << argv[0]
                          << " --virtual --left_sn 301528071 --right_sn 300473441"
                             " --calib_opencv SN.yml"
                          << std::endl;
                std::cout << std::endl;
                exit(0);
            }
        }
    }
};

// Ctrl+C / SIGTERM: request a clean shutdown (release camera, exit). A second
// signal falls back to the default handler and force-kills.
std::atomic<bool> g_stop_requested{false};
void onStopSignal(int sig) {
    g_stop_requested = true;
    std::signal(sig, SIG_DFL);
}

// ---------------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    std::signal(SIGINT, onStopSignal);
    std::signal(SIGTERM, onStopSignal);

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

    std::cout << "*** Stereo Calibration Checker ***" << std::endl << std::endl;
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

    // ---- ZED camera initialization (mirrors stereo_calibration) ----
    sl::Camera zed_camera;
    sl::InitParameters init_params;
    init_params.depth_mode = sl::DEPTH_MODE::NONE;
    init_params.camera_resolution = sl::RESOLUTION::AUTO;
    init_params.camera_fps = 15;
    init_params.enable_image_validity_check = false;
    init_params.camera_disable_self_calib = true;
    init_params.sdk_verbose = sdk_verbose;

    if (!args.svo_path.empty())
        init_params.input.setFromSVOFile(args.svo_path.c_str());

    if (!args.calib_opencv_file.empty())
        init_params.optional_opencv_calibration_file = args.calib_opencv_file.c_str();

    if (args.is_zed_x_one_virtual_stereo) {
        std::cout << " * Virtual Stereo Camera mode enabled." << std::endl;
        int sn_left = args.left_camera_sn;
        int sn_right = args.right_camera_sn;

        if (sn_left != -1 && sn_right != -1) {
            int sn_stereo = sl::generateVirtualStereoSerialNumber(sn_left, sn_right);
            std::cout << " * Unique Virtual SN: " << sn_stereo << std::endl;
            init_params.input.setVirtualStereoFromSerialNumbers(sn_left, sn_right, sn_stereo);
        } else if (args.left_camera_id != -1 && args.right_camera_id != -1) {
            auto cams = sl::CameraOne::getDeviceList();
            for (auto& cam : cams) {
                if (cam.id == args.left_camera_id) sn_left = cam.serial_number;
                else if (cam.id == args.right_camera_id) sn_right = cam.serial_number;
            }
            if (sn_left == -1 || sn_right == -1) {
                std::cerr << "Error: Could not find serial numbers for provided camera IDs." << std::endl;
                std::cerr << " * Use 'ZED_Explore --all' to list connected cameras." << std::endl;
                return EXIT_FAILURE;
            }
            int sn_stereo = sl::generateVirtualStereoSerialNumber(sn_left, sn_right);
            std::cout << " * Unique Virtual SN: " << sn_stereo << std::endl;
            init_params.input.setVirtualStereoFromCameraIDs(
                args.left_camera_id, args.right_camera_id, sn_stereo);
        } else {
            std::cerr << "Error: --virtual requires --left_id/--right_id or --left_sn/--right_sn." << std::endl;
            return EXIT_FAILURE;
        }

        int left_model = sn_left / 10000000;
        if (left_model == static_cast<int>(sl::MODEL::ZED_XONE_UHD)) {
            init_params.camera_resolution = sl::RESOLUTION::HD4K;
            std::cout << " * ZED X One 4K Virtual Stereo Camera detected." << std::endl;
        } else {
            init_params.camera_resolution = sl::RESOLUTION::HD1200;
            std::cout << " * ZED X One GS Virtual Stereo Camera detected." << std::endl;
        }
    }

    auto status = zed_camera.open(init_params);
    // Tolerate a missing/invalid on-camera calibration: a valid --calib_opencv
    // file (or the checker's own board detection) can still drive the tool.
    if (status > sl::ERROR_CODE::SUCCESS &&
        status != sl::ERROR_CODE::INVALID_CALIBRATION_FILE) {
        std::cerr << "Error opening ZED camera: " << sl::toString(status) << " - " << sl::toVerbose(status) << std::endl;
        return EXIT_FAILURE;
    }

    auto zed_info = zed_camera.getCameraInformation();
    sl::Resolution cam_res = zed_info.camera_configuration.resolution;
    cv::Size full_size(cam_res.width, cam_res.height);

    std::cout << " * Camera Model: " << sl::toString(zed_info.camera_model) << std::endl;
    std::cout << " * Camera S/N: " << zed_info.serial_number << std::endl;
    std::cout << " * Camera Resolution: " << full_size.width << " x " << full_size.height << std::endl;

    // ---- Populate calibration from the SDK (reflects the active calibration source) ----
    StereoCalibData calib;
    calib.setFrom(zed_info.camera_configuration.calibration_parameters_raw);

    const std::string calib_source = args.calib_opencv_file.empty()
                                         ? "Internal ZED calibration"
                                         : "OpenCV file: " + args.calib_opencv_file;
    std::cout << " * Calibration source: " << calib_source << std::endl;
    std::cout << " * Distortion model: "
              << (calib.is_fisheye ? "Fisheye" : "Radial-Tangential") << std::endl << std::endl;

    sl::Mat zed_imgL(cam_res, sl::MAT_TYPE::U8_C3, sl::MEM::CPU);
    sl::Mat zed_imgR(cam_res, sl::MAT_TYPE::U8_C3, sl::MEM::CPU);
    cv::Mat rgb_l(full_size.height, full_size.width, CV_8UC3, zed_imgL.getPtr<sl::uchar1>());
    cv::Mat rgb_r(full_size.height, full_size.width, CV_8UC3, zed_imgR.getPtr<sl::uchar1>());

    // 3D points of the checkerboard pattern
    std::vector<cv::Point3f> obj_pts;
    for (int i = 0; i < v_edges; i++)
        for (int j = 0; j < h_edges; j++)
            obj_pts.push_back(cv::Point3f(square_size * j, square_size * i, 0.f));

    // Sub-pixel refinement window: larger for higher resolution
    const int subpix_win = (full_size.width >= 3000) ? 15 : 11;

    // ---- Display setup ----
    const std::string window_name = "ZED Stereo Checker";
    cv::namedWindow(window_name, cv::WINDOW_KEEPRATIO);
    cv::resizeWindow(window_name, display_size.width * 2,
                     display_size.height + text_area_height);

    // Keep last valid error values so the display does not flicker when the
    // target temporarily disappears
    double disp_left = -1.0, disp_right = -1.0, disp_stereo = -1.0;

    std::cout << "Point the camera at the checkerboard. Press 'q' or ESC to quit." << std::endl;

    char key = ' ';
    bool window_was_visible = false;  // arms the title-bar-X close detection
    while (1) {
        if (key == 'q' || key == 'Q' || key == 27 || g_stop_requested) break;

        // Window closed via the title-bar X: quit like 'q'. The GTK backend
        // does not implement WND_PROP_VISIBLE, but any property query returns
        // -1 once the window is destroyed — probe WND_PROP_AUTOSIZE (>= 0
        // while the window exists), armed after the first positive read.
        const bool window_exists =
            cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0;
        if (window_exists) {
            window_was_visible = true;
        } else if (window_was_visible) {
            break;
        }

        if (zed_camera.grab() != sl::ERROR_CODE::SUCCESS) {
            key = cv::waitKey(10);
            continue;
        }

        zed_camera.retrieveImage(zed_imgL, sl::VIEW::LEFT_UNRECTIFIED_BGR);
        zed_camera.retrieveImage(zed_imgR, sl::VIEW::RIGHT_UNRECTIFIED_BGR);

        cv::Mat rgb_d_l, rgb_d_r;
        cv::resize(rgb_l, rgb_d_l, display_size);
        cv::resize(rgb_r, rgb_d_r, display_size);

        double err_left = -1.0, err_right = -1.0, err_stereo = -1.0;
        bool found_l = false, found_r = false;

        if (use_charuco) {
            // ---- ChArUco: full-resolution, id-tagged, partial views OK ----
            BoardDetection det_l = detector.detect(rgb_l);
            BoardDetection det_r = detector.detect(rgb_r);
            found_l = det_l.found;
            found_r = det_r.found;

            if (found_l) {
                cv::Mat rvec_l, tvec_l;
                if (computePose(det_l.objectPoints, det_l.imagePoints,
                                calib.K_left, calib.D_left, calib.is_fisheye,
                                rvec_l, tvec_l)) {
                    err_left = computeReprojError(det_l.objectPoints,
                                                  det_l.imagePoints, rvec_l,
                                                  tvec_l, calib.K_left,
                                                  calib.D_left, calib.is_fisheye);
                }
            }
            if (found_r) {
                cv::Mat rvec_r_ind, tvec_r_ind;
                if (computePose(det_r.objectPoints, det_r.imagePoints,
                                calib.K_right, calib.D_right, calib.is_fisheye,
                                rvec_r_ind, tvec_r_ind)) {
                    err_right = computeReprojError(
                        det_r.objectPoints, det_r.imagePoints, rvec_r_ind,
                        tvec_r_ind, calib.K_right, calib.D_right,
                        calib.is_fisheye);
                }
            }

            // Draw downscaled copies on the display images: corner dots when
            // the board is valid, raw ArUco tag outlines always (shows which
            // markers each camera picks up even when the board detection
            // fails in that view).
            auto draw_view = [&](const BoardDetection& det, cv::Mat& img) {
                BoardDetection dd = det;
                scaleKP(dd.imagePoints, full_size, display_size);
                for (auto& mc : dd.markerCorners)
                    scaleKP(mc, full_size, display_size);
                detector.draw(img, dd);
            };
            draw_view(det_l, rgb_d_l);
            draw_view(det_r, rgb_d_r);

            // Mirror the preview (selfie view) AFTER scene-space overlays,
            // BEFORE the HUD text so it stays readable.
            if (args.mirror) {
                cv::flip(rgb_d_l, rgb_d_l, 1);
                cv::flip(rgb_d_r, rgb_d_r, 1);
            }

            // ChArUco tuning HUD: detected corners per view vs full grid.
            {
                const int total = detector.cornersX() * detector.cornersY();
                auto draw_hud = [&](cv::Mat& img, size_t n) {
                    std::stringstream ss_ch;
                    ss_ch << "ChArUco: " << n << "/" << total;
                    cv::putText(img, ss_ch.str(), cv::Point(10, 70),
                                cv::FONT_HERSHEY_SIMPLEX, 0.55,
                                cv::Scalar(0, 0, 0), 3);
                    cv::putText(img, ss_ch.str(), cv::Point(10, 70),
                                cv::FONT_HERSHEY_SIMPLEX, 0.55,
                                cv::Scalar(50, 210, 50), 1);
                };
                draw_hud(rgb_d_l, det_l.ids.size());
                draw_hud(rgb_d_r, det_r.ids.size());
            }

            // Stereo error: use only corners whose id is seen in BOTH views.
            if (found_l && found_r) {
                std::unordered_map<int, cv::Point2f> right_by_id;
                for (size_t k = 0; k < det_r.ids.size(); k++)
                    right_by_id.emplace(det_r.ids[k], det_r.imagePoints[k]);

                std::vector<cv::Point3f> obj_common;
                std::vector<cv::Point2f> l_common, r_common;
                for (size_t k = 0; k < det_l.ids.size(); k++) {
                    auto it = right_by_id.find(det_l.ids[k]);
                    if (it == right_by_id.end()) continue;
                    obj_common.push_back(det_l.objectPoints[k]);
                    l_common.push_back(det_l.imagePoints[k]);
                    r_common.push_back(it->second);
                }

                if ((int)obj_common.size() >= 4) {
                    cv::Mat rvl, tvl;
                    if (computePose(obj_common, l_common, calib.K_left,
                                    calib.D_left, calib.is_fisheye, rvl, tvl)) {
                        double eL = computeReprojError(obj_common, l_common, rvl,
                                                       tvl, calib.K_left,
                                                       calib.D_left,
                                                       calib.is_fisheye);
                        cv::Mat R_l, R_r, t_r, rvec_r_stereo;
                        cv::Rodrigues(rvl, R_l);
                        R_r = calib.R * R_l;
                        t_r = calib.R * tvl + calib.T;
                        cv::Rodrigues(R_r, rvec_r_stereo);
                        double eRs = computeReprojError(
                            obj_common, r_common, rvec_r_stereo, t_r,
                            calib.K_right, calib.D_right, calib.is_fisheye);
                        err_stereo = std::sqrt((eL * eL + eRs * eRs) / 2.0);
                    }
                }
            }
        } else {
            // ---- Chessboard detection on downscaled images (fast) ----
            std::vector<cv::Point2f> pts_l_d, pts_r_d;
            found_l = cv::findChessboardCorners(rgb_d_l, cv::Size(h_edges, v_edges), pts_l_d);
            found_r = cv::findChessboardCorners(rgb_d_r, cv::Size(h_edges, v_edges), pts_r_d);

            cv::Mat rvec_l, tvec_l;
            std::vector<cv::Point2f> pts_l_f, pts_r_f;

            if (found_l) {
                // Scale corners to full resolution and refine sub-pixel
                pts_l_f = pts_l_d;
                scaleKP(pts_l_f, display_size, full_size);
                cv::Mat gray_l;
                cv::cvtColor(rgb_l, gray_l, cv::COLOR_BGR2GRAY);
                cv::cornerSubPix(gray_l, pts_l_f, cv::Size(subpix_win, subpix_win),
                                 cv::Size(-1, -1),
                                 cv::TermCriteria(cv::TermCriteria::EPS |
                                                      cv::TermCriteria::MAX_ITER,
                                                  30, 0.001));

                // Left reprojection error: tests left camera intrinsics (K_left, D_left)
                if (computePose(obj_pts, pts_l_f, calib.K_left, calib.D_left,
                                calib.is_fisheye, rvec_l, tvec_l)) {
                    err_left = computeReprojError(obj_pts, pts_l_f, rvec_l, tvec_l,
                                                 calib.K_left, calib.D_left,
                                                 calib.is_fisheye);
                }

                cv::drawChessboardCorners(rgb_d_l, cv::Size(h_edges, v_edges),
                                          cv::Mat(pts_l_d), found_l);
            }

            if (found_r) {
                // Scale corners to full resolution and refine sub-pixel
                pts_r_f = pts_r_d;
                scaleKP(pts_r_f, display_size, full_size);
                cv::Mat gray_r;
                cv::cvtColor(rgb_r, gray_r, cv::COLOR_BGR2GRAY);
                cv::cornerSubPix(gray_r, pts_r_f, cv::Size(subpix_win, subpix_win),
                                 cv::Size(-1, -1),
                                 cv::TermCriteria(cv::TermCriteria::EPS |
                                                      cv::TermCriteria::MAX_ITER,
                                                  30, 0.001));

                // Right reprojection error: tests right camera intrinsics independently
                cv::Mat rvec_r_ind, tvec_r_ind;
                if (computePose(obj_pts, pts_r_f, calib.K_right, calib.D_right,
                                calib.is_fisheye, rvec_r_ind, tvec_r_ind)) {
                    err_right = computeReprojError(obj_pts, pts_r_f, rvec_r_ind,
                                                  tvec_r_ind, calib.K_right,
                                                  calib.D_right, calib.is_fisheye);
                }

                cv::drawChessboardCorners(rgb_d_r, cv::Size(h_edges, v_edges),
                                          cv::Mat(pts_r_d), found_r);
            }

            // Stereo reprojection error: uses left pose + stereo extrinsics (R, T)
            // to predict right image corners — tests the extrinsic calibration
            if (found_l && found_r && err_left >= 0.0) {
                cv::Mat R_l, R_r, t_r, rvec_r_stereo;
                cv::Rodrigues(rvec_l, R_l);
                R_r = calib.R * R_l;
                t_r = calib.R * tvec_l + calib.T;
                cv::Rodrigues(R_r, rvec_r_stereo);

                double err_right_from_stereo =
                    computeReprojError(obj_pts, pts_r_f, rvec_r_stereo, t_r,
                                       calib.K_right, calib.D_right, calib.is_fisheye);

                // Combined stereo RMS (same formula as in stereo calibration)
                err_stereo = std::sqrt(
                    (err_left * err_left + err_right_from_stereo * err_right_from_stereo) / 2.0);
            }

            if (args.mirror) {
                cv::flip(rgb_d_l, rgb_d_l, 1);
                cv::flip(rgb_d_r, rgb_d_r, 1);
            }
        }

        // Update persisted display values when detection is valid
        if (err_left >= 0.0) disp_left = err_left;
        if (err_right >= 0.0) disp_right = err_right;
        if (err_stereo >= 0.0) disp_stereo = err_stereo;

        // Reset stored values when the target disappears
        if (!found_l) disp_left = -1.0;
        if (!found_r) disp_right = -1.0;
        if (!found_l || !found_r) disp_stereo = -1.0;

        // ---- Draw error values directly on the images ----
        drawErrorText(rgb_d_l, "Left reproj.", disp_left, cv::Point(10, 35));
        drawErrorText(rgb_d_r, "Right reproj.", disp_right, cv::Point(10, 35));

        // ---- Compose final display ----
        cv::Mat display;
        if (image_stack_horizontal)
            cv::hconcat(rgb_d_l, rgb_d_r, display);
        else
            cv::vconcat(rgb_d_l, rgb_d_r, display);

        cv::Mat text_area =
            cv::Mat::ones(cv::Size(display.cols, text_area_height), display.type());
        cv::vconcat(display, text_area, display);

        const int ty_base = display_size.height + 40;
        drawErrorText(display, "Stereo reproj.", disp_stereo, cv::Point(10, ty_base));

        // Color legend
        const int ly = ty_base + 38;
        cv::putText(display, "< 0.5 px: good", cv::Point(10, ly),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50, 210, 50), 1);
        cv::putText(display, "0.5-1.0 px: acceptable", cv::Point(190, ly),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 165, 255), 1);
        cv::putText(display, "> 1.0 px: poor", cv::Point(440, ly),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 50, 250), 1);

        // Calibration source label
        cv::putText(display, "Calib: " + calib_source,
                    cv::Point(10, display_size.height + text_area_height - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(150, 150, 150), 1);

        cv::putText(display, "Press 'q' / ESC to quit",
                    cv::Point(display.cols - 280,
                              display_size.height + text_area_height - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(150, 150, 150), 1);

        cv::imshow(window_name, display);
        key = static_cast<char>(cv::waitKey(10));
    }

    zed_camera.close();
    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}
