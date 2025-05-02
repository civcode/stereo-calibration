// #include <stdio.h>
#include <iostream>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cxxopts.hpp"

#include "calibration/types.hpp"

using std::cout;
using std::endl;

std::vector<std::vector<cv::Point3f>> object_points;
std::vector<std::vector<cv::Point2f>> imagePoints1, imagePoints2;
std::vector<cv::Point2f> corners1, corners2;
std::vector<std::vector<cv::Point2f>> left_img_points, right_img_points;


void GetImagePoints(int board_width, int board_height, float square_size,
                    const std::vector<std::string>& image_names, const VisualizationConfig& config) {

  if (config.visualize) {
    cv::namedWindow("Corners 1", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Corners 2", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("Corners 1", 100, 100);
    cv::moveWindow("Corners 2", 200, 100);
  }

  cv::Mat gray1;
  cv::Mat gray2;
  cv::Size board_size = cv::Size(board_width, board_height);

  for (auto & name : image_names) {
    cout << "Reading image: " << name << endl;
    cv::Mat stereo_img = cv::imread(name, cv::IMREAD_COLOR);

    if (stereo_img.empty()) {
      cout << "Error reading image: " << name << endl;
      continue;
    }
    cv::Mat img1 = stereo_img(cv::Rect(0, 0, stereo_img.cols / 2, stereo_img.rows));
    cv::Mat img2 = stereo_img(cv::Rect(stereo_img.cols / 2, 0, stereo_img.cols / 2, stereo_img.rows));

    cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);

    int flags = (cv::CALIB_CB_ADAPTIVE_THRESH |
                  cv::CALIB_CB_NORMALIZE_IMAGE |
                  cv::CALIB_CB_FAST_CHECK |
                  cv::CALIB_CB_FILTER_QUADS);

    bool found1 = cv::findChessboardCorners(img1, board_size, corners1, flags);
    bool found2 = cv::findChessboardCorners(img2, board_size, corners2, flags);

    if (!found1 || !found2) {
      cout << "Could not find chessboard corners" << endl;
      continue;
    }

    auto criteria = cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 100, 1E-5);

    cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1), criteria);
    cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1), criteria);

    if (config.visualize) {
      cv::drawChessboardCorners(img1, board_size, corners1, found1);
      cv::drawChessboardCorners(img2, board_size, corners2, found2);
      cv::imshow("Corners 1", img1);
      cv::imshow("Corners 2", img2);
      cv::waitKey(config.wait_time);
    }

    std::vector<cv::Point3f> obj;
    for (int i = 0; i < board_height; i++) {
      for (int j = 0; j < board_width; j++) {
        obj.push_back(cv::Point3f(j * square_size, i * square_size, 0));
      }
    }

    if (found1 && found2) {
      imagePoints1.push_back(corners1);
      imagePoints2.push_back(corners2);
      object_points.push_back(obj);
    }
  }

  for (size_t i = 0; i < imagePoints1.size(); i++) {
    std::vector<cv::Point2f> v1, v2;
    for (size_t j = 0; j < imagePoints1[i].size(); j++) {
      v1.push_back(cv::Point2f((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
      v2.push_back(cv::Point2f((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
    }
    left_img_points.push_back(v1);
    right_img_points.push_back(v2);
  }
}

int main(int argc, char* argv[]) {

  std::string left_calib_file;
  std::string right_calib_file;
  std::string stereo_img_file;
  std::string out_file;

  VisualizationConfig config;

  try {
    cxxopts::Options options(argv[0], "Stereo camera calibration using checkerboard images");
    options.add_options()
      ("l,left_calib_file", "Left camera calibration file", cxxopts::value<std::string>(left_calib_file))
      ("r,right_calib_file", "Right camera calibration file", cxxopts::value<std::string>(right_calib_file))
      ("s,stereo_img_file", "Stereo image file list", cxxopts::value<std::string>(stereo_img_file))
      ("o,out_file", "Output calibration filename (YML)", cxxopts::value<std::string>(out_file)->default_value("stereo_calib.json"))
      ("v,visualize", "Visualize corners", cxxopts::value<bool>()->default_value("false"))
      ("d,delay", "Wait time for visualization in ms", cxxopts::value<int>()->default_value("500"))
      ("help", "Print help");

    auto result = options.parse(argc, argv);

    config = {.visualize = result["visualize"].as<bool>(),
              .wait_time = result["delay"].as<int>()};

    if (result.count("help")) {
      cout << options.help() << endl;
      return 0;
    }

  } catch (const cxxopts::OptionException& e) {
    cout << "Error parsing options: " << e.what() << endl;
    return 1;
  }

  cv::FileStorage fsl(left_calib_file, cv::FileStorage::READ);
  cv::FileStorage fsr(right_calib_file, cv::FileStorage::READ);

  int board_width = static_cast<int>(fsl["board_width"]);
  int board_height = static_cast<int>(fsl["board_height"]);
  float square_size = static_cast<float>(fsl["square_size"]);

  cout << "Board width: " << board_width << endl;
  cout << "Board height: " << board_height << endl;
  cout << "Square size: " << square_size << endl;

  std::vector<std::string> image_names;
  {
    cv::FileStorage fs(stereo_img_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      cout << "Error opening file: " << stereo_img_file << endl;
      return -1;
    }
    std::vector<std::string> name_list;
    fs["image_names"] >> name_list;
    fs.release();
    for (const auto& name : name_list) {
      if (!std::filesystem::exists(name)) {
        cout << "File does not exist: " << name << endl;
      } else {
        cout << "File exists: " << name << endl;
        image_names.push_back(name);
      }
    }
  }
  {
    cv::FileStorage fs(stereo_img_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      cout << "Error opening file: " << stereo_img_file << endl;
      return -1;
    }
    std::vector<std::string> name_list;
    fs["image_names"] >> name_list;
    fs.release();
    //check if files exits
    for (const auto& name : name_list) {
      if (!std::filesystem::exists(name)) {
        cout << "File does not exist: " << name << endl;
      } else {
        cout << "File exists: " << name << endl;
        image_names.push_back(name);
      }
    }
  }

  GetImagePoints(board_width, board_height, square_size, image_names, config);

  cout << "Starting Calibration" << endl;
  cv::Mat K1, K2, R, F, E;
  cv::Vec3d T;
  cv::Mat D1, D2;

  fsl["K"] >> K1;
  fsr["K"] >> K2;
  fsl["D"] >> D1;
  fsr["D"] >> D2;


  cout << "object_points size: " << object_points.size() << endl;
  cout << "left_image_points size: " << left_img_points.size() << endl;
  cout << "right_image_points size: " << right_img_points.size() << endl;


  cv::Mat stereo_img = cv::imread(image_names[0]);
  if (stereo_img.empty()) {
      std::cerr << "Failed to load reference image to get image size!" << std::endl;
      return -1;
  }
  cv::Mat img = stereo_img(cv::Rect(0, 0, stereo_img.cols / 2, stereo_img.rows));
  cv::Mat per_view_errors;

  // int flags = (cv::CALIB_FIX_INTRINSIC);

  int flags = (cv::CALIB_USE_INTRINSIC_GUESS |
				        cv::CALIB_FIX_ASPECT_RATIO |
                cv::CALIB_ZERO_TANGENT_DIST |
                cv::CALIB_SAME_FOCAL_LENGTH |
                cv::CALIB_RATIONAL_MODEL |
                cv::CALIB_FIX_K3 |
                cv::CALIB_FIX_K4 |
                cv::CALIB_FIX_K5);

  auto criteria = cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 100, 1E-5);

  double rms = cv::stereoCalibrate(object_points, left_img_points, right_img_points, K1, D1, K2, D2,
                      img.size(), R, T, E, F, per_view_errors, flags, criteria);

  cout << "img size: " << img.size() << endl;
  cout << "RMS error: " << rms << endl;

  cv::FileStorage fs1(out_file, cv::FileStorage::WRITE);
  fs1 << "K1" << K1;
  fs1 << "K2" << K2;
  fs1 << "D1" << D1;
  fs1 << "D2" << D2;
  fs1 << "R" << R;
  fs1 << "T" << T;
  fs1 << "E" << E;
  fs1 << "F" << F;

  cout << "K1: " << K1 << endl;
  cout << "K2: " << K2 << endl;
  cout << "D1: " << D1 << endl;
  cout << "D2: " << D2 << endl;
  cout << "R: " << R << endl;
  cout << "T: " << T << endl;
  cout << "E: " << E << endl;
  cout << "F: " << F << endl;


  cout << "Done Calibration" << endl;

  cout << "Starting Rectification" << endl;

  cv::Mat R1, R2, P1, P2, Q;

  cv::stereoRectify(K1, D1, K2, D2, img.size(), R, T, R1, R2, P1, P2, Q, 0, 0);

  fs1 << "R1" << R1;
  fs1 << "R2" << R2;
  fs1 << "P1" << P1;
  fs1 << "P2" << P2;
  fs1 << "Q" << Q;

  cout << "R1: " << R1 << endl;
  cout << "R2: " << R2 << endl;
  cout << "P1: " << P1 << endl;
  cout << "P2: " << P2 << endl;
  cout << "Q: " << Q << endl;

  cout << "Done Rectification" << endl;

  return 0;
}
