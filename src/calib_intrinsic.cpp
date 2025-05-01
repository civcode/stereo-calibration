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

using std::cout;
using std::endl;

std::vector<std::vector<cv::Point3f>> object_points;
std::vector<std::vector<cv::Point2f>> image_points;
std::vector<cv::Point2f> corners;
std::vector<std::vector<cv::Point2f>> left_img_points;

cv::Mat img;
cv::Mat gray;
cv::Size img_size;

void GetImagePoints(int board_width, int board_height, std::vector<std::string>& image_names,
                       float square_size, const std::string& imgs_filename, const std::string& extension) {
  cv::Size board_size = cv::Size(board_width, board_height);
  int board_n = board_width * board_height;

  for (auto & img_file : image_names) {
    cout << "Reading image: " << img_file << endl;
    img = cv::imread(img_file, cv::IMREAD_COLOR);
    if (img.empty()) {
      cout << "Error reading image: " << img_file << endl;
      continue;
    }
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    bool found = cv::findChessboardCorners(img, board_size, corners,
                                           cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
    if (found) {
      cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                       cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));
      cv::drawChessboardCorners(gray, board_size, corners, found);
    }

    std::vector<cv::Point3f> obj;
    for (int i = 0; i < board_height; i++)
      for (int j = 0; j < board_width; j++)
        obj.push_back(cv::Point3f((float)j * square_size, (float)i * square_size, 0));

    if (found) {
      image_points.push_back(corners);
      object_points.push_back(obj);
    }
  }
}

double ComputeReprojectionErrors(const std::vector<std::vector<cv::Point3f>>& objectPoints,
                                 const std::vector<std::vector<cv::Point2f>>& imagePoints,
                                 const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                 const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
  std::vector<cv::Point2f> imagePoints2;
  int totalPoints = 0;
  double totalErr = 0;
  double err;
  std::vector<float> perViewErrors(objectPoints.size());

  for (size_t i = 0; i < objectPoints.size(); ++i) {
    cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix,
                      distCoeffs, imagePoints2);
    err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float)std::sqrt(err * err / n);
    totalErr += err * err;
    totalPoints += n;
  }
  return std::sqrt(totalErr / totalPoints);
}

int main(int argc, char* argv[]) {
  int board_width;
  int board_height;
  int num_imgs;
  float square_size;
  std::string imgs_directory;
  std::string imgs_filename;
  std::string out_file;
  std::string extension;
  std::string image_file_list;

  try {
    cxxopts::Options options(argv[0], "Camera calibration using checkerboard images");
    options.add_options()
      ("w,board_width", "Checkerboard width", cxxopts::value<int>(board_width)->default_value("9"))
      ("h,board_height", "Checkerboard height", cxxopts::value<int>(board_height)->default_value("6"))
      ("s,square_size", "Size of checkerboard square", cxxopts::value<float>(square_size)->default_value("0.025"))
      // ("d,imgs_directory", "Directory containing images", cxxopts::value<std::string>(imgs_directory)->default_value("./img"))
      ("i,imgs_filename", "Image filename", cxxopts::value<std::string>(imgs_filename)->default_value("left"))
      ("e,extension", "Image extension", cxxopts::value<std::string>(extension)->default_value(".png"))
      // ("o,out_file", "Output calibration filename (xml/json/yml)", cxxopts::value<std::string>(out_file)->default_value("calibration.json"))
      ("l,list", "File lsit", cxxopts::value<std::string>(image_file_list)->default_value("img-left-meta.json"))
      ("help", "Print help");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      cout << options.help() << endl;
      return 0;
    }

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

  // out_file = "calib-cam-" + std::to_string(cam_id) + ".json";
  out_file = "cam-" + std::to_string(cam_id) + "-calib.json";

  GetImagePoints(board_width, board_height, image_names, square_size,
                    imgs_filename, extension);

  cout << "Starting Calibration" << endl;
  cv::Mat K, D;
  std::vector<cv::Mat> rvecs, tvecs;
  int flag = 0;
  flag |= cv::CALIB_FIX_K4;
  flag |= cv::CALIB_FIX_K5;
  double reprojection_error = cv::calibrateCamera(object_points, image_points, img.size(), K, D, rvecs, tvecs, flag);
  cout << "Reprojection error: " << reprojection_error << endl;
  // cout << "Calibration error: " << ComputeReprojectionErrors(object_points, image_points, rvecs, tvecs, K, D) << endl;
  double alpha = 0;
  cv::Mat new_K = cv::getOptimalNewCameraMatrix(K, D, img.size(), alpha, img.size());
  cout << "New camera matrix: " << new_K << endl;

  cv::FileStorage fs(out_file, cv::FileStorage::WRITE);
  // fs << "K" << K;
  fs << "K" << new_K;
  fs << "D" << D;
  fs << "board_width" << board_width;
  fs << "board_height" << board_height;
  fs << "square_size" << square_size;
  cout << "Done Calibration" << endl;

  return 0;
}
