// #include <stdio.h>
#include <filesystem>
#include <iostream>
#include <vector>
// #include <sys/stat.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cxxopts.hpp"

#include "calibration/types.hpp"

using std::cout;
using std::endl;

std::vector<std::vector<cv::Point3f>> object_points;
std::vector<std::vector<cv::Point2f>> image_points;
std::vector<cv::Point2f> corners;
std::vector<std::vector<cv::Point2f>> left_img_points;

void GetImagePoints(int board_width, int board_height, float square_size,
  const std::vector<std::string>& image_names, const VisualizationConfig& config) {

  cv::Size board_size = cv::Size(board_width, board_height);

  for (auto & img_file : image_names) {
    cout << "Reading image: " << img_file << endl;
    cv::Mat img = cv::imread(img_file, cv::IMREAD_COLOR);
    if (img.empty()) {
      cout << "Error reading image: " << img_file << endl;
      continue;
    }
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    int flags = (cv::CALIB_CB_ADAPTIVE_THRESH |
                  cv::CALIB_CB_NORMALIZE_IMAGE |
                  cv::CALIB_CB_FAST_CHECK |
                  cv::CALIB_CB_FILTER_QUADS);

    bool found = cv::findChessboardCorners(img, board_size, corners, flags);

    if (found) {
      auto criteria = cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 100, 1E-5);
      cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
      if (config.visualize) {
        cv::drawChessboardCorners(img, board_size, corners, found);
        cv::imshow("Corners", img);
        cv::waitKey(config.wait_time);
      }
    }

    std::vector<cv::Point3f> obj;
    for (int i = 0; i < board_height; i++)
      for (int j = 0; j < board_width; j++)
        obj.push_back(cv::Point3f(static_cast<float>(j) * square_size, static_cast<float>(i) * square_size, 0));

    if (found) {
      image_points.push_back(corners);
      object_points.push_back(obj);
    }
  }
}

int main(int argc, char* argv[]) {
  int board_width;
  int board_height;
  float square_size;
  std::string out_file;
  std::string image_file_list;

  VisualizationConfig config;

  try {
    cxxopts::Options options(argv[0], "Camera calibration using checkerboard images");
    options.add_options()
      ("w,board_width", "Checkerboard width", cxxopts::value<int>(board_width)->default_value("9"))
      ("h,board_height", "Checkerboard height", cxxopts::value<int>(board_height)->default_value("6"))
      ("s,square_size", "Size of checkerboard square", cxxopts::value<float>(square_size)->default_value("0.025"))
      ("l,list", "File lsit", cxxopts::value<std::string>(image_file_list)->default_value("img-left-meta.json"))
      ("v,visualize", "Visualize corners", cxxopts::value<bool>()->default_value("false"))
      ("d,delay", "Wait time for visualization in ms", cxxopts::value<int>()->default_value("500"))
      ("help", "Print help");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      cout << options.help() << endl;
      return 0;
    }

    config = {.visualize = result["visualize"].as<bool>(),
              .wait_time = result["delay"].as<int>()};

  } catch (const cxxopts::OptionException& e) {
    cout << "Error parsing options: " << e.what() << endl;
    return 1;
  }

  int cam_id;
  std::vector<std::string> image_names;
  {
    cv::FileStorage fs(image_file_list, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      cout << "Error opening file: " << image_file_list << endl;
      return -1;
    }
    std::vector<std::string> name_list;
    fs["camera_id"] >> cam_id;
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

  out_file = "cam-" + std::to_string(cam_id) + "-calib.json";

  GetImagePoints(board_width, board_height, square_size, image_names, config);

  cout << "Starting Calibration" << endl;

  cv::Mat img = cv::imread(image_names[0], cv::IMREAD_COLOR);
  cv::Mat K, D;
  std::vector<cv::Mat> rvecs, tvecs;

  int flags = (cv::CALIB_FIX_K4 |
              cv::CALIB_FIX_K5);

  auto criteria = cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 100, 1E-5);

  double rms = cv::calibrateCamera(object_points, image_points, img.size(), K, D, rvecs, tvecs, flags, criteria);
  cout << "rms error: " << rms << endl;

  double alpha = 0;
  cv::Mat new_K = cv::getOptimalNewCameraMatrix(K, D, img.size(), alpha, img.size());
  cout << "New camera matrix: " << new_K << endl;

  cv::FileStorage fs(out_file, cv::FileStorage::WRITE);
  fs << "K" << new_K;
  fs << "D" << D;
  fs << "board_width" << board_width;
  fs << "board_height" << board_height;
  fs << "square_size" << square_size;
  cout << "Done Calibration" << endl;

  return 0;
}
