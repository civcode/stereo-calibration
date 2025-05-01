#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <sys/stat.h>
#include "cxxopts.hpp"

using std::cout;
using std::endl;

std::vector<std::vector<cv::Point3f>> object_points;
std::vector<std::vector<cv::Point2f>> image_points;
std::vector<cv::Point2f> corners;
std::vector<std::vector<cv::Point2f>> left_img_points;

cv::Mat img, gray;
cv::Size im_size;

bool doesExist(const std::string& name) {
  struct stat buffer;
  return (stat(name.c_str(), &buffer) == 0);
}

void setup_calibration(int board_width, int board_height, int num_imgs,
                       float square_size, const std::string& imgs_directory,
                       const std::string& imgs_filename, const std::string& extension) {
  cv::Size board_size = cv::Size(board_width, board_height);
  int board_n = board_width * board_height;

  for (int k = 1; k <= num_imgs; k++) {
    char img_file[100];
    sprintf(img_file, "%s%s%d.%s", imgs_directory.c_str(), imgs_filename.c_str(), k, extension.c_str());
    if (!doesExist(img_file))
      continue;
    img = cv::imread(img_file, cv::IMREAD_COLOR);
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
      cout << k << ". Found corners!" << endl;
      image_points.push_back(corners);
      object_points.push_back(obj);
    }
  }
}

double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>>& objectPoints,
                                 const std::vector<std::vector<cv::Point2f>>& imagePoints,
                                 const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                 const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
  std::vector<cv::Point2f> imagePoints2;
  int totalPoints = 0;
  double totalErr = 0, err;
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

int main(int argc, char** argv) {
  int board_width, board_height, num_imgs;
  float square_size;
  std::string imgs_directory, imgs_filename, out_file, extension;

  try {
    cxxopts::Options options(argv[0], "Camera calibration using checkerboard images");
    options.add_options()
      ("w,board_width", "Checkerboard width", cxxopts::value<int>(board_width))
      ("h,board_height", "Checkerboard height", cxxopts::value<int>(board_height))
      ("n,num_imgs", "Number of checkerboard images", cxxopts::value<int>(num_imgs))
      ("s,square_size", "Size of checkerboard square", cxxopts::value<float>(square_size))
      ("d,imgs_directory", "Directory containing images", cxxopts::value<std::string>(imgs_directory))
      ("i,imgs_filename", "Image filename", cxxopts::value<std::string>(imgs_filename))
      ("e,extension", "Image extension", cxxopts::value<std::string>(extension))
      ("o,out_file", "Output calibration filename (YML)", cxxopts::value<std::string>(out_file))
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

  setup_calibration(board_width, board_height, num_imgs, square_size,
                    imgs_directory, imgs_filename, extension);

  cout << "Starting Calibration" << endl;
  cv::Mat K, D;
  std::vector<cv::Mat> rvecs, tvecs;
  int flag = 0;
  flag |= cv::CALIB_FIX_K4;
  flag |= cv::CALIB_FIX_K5;
  cv::calibrateCamera(object_points, image_points, img.size(), K, D, rvecs, tvecs, flag);

  cout << "Calibration error: " << computeReprojectionErrors(object_points, image_points, rvecs, tvecs, K, D) << endl;

  cv::FileStorage fs(out_file, cv::FileStorage::WRITE);
  fs << "K" << K;
  fs << "D" << D;
  fs << "board_width" << board_width;
  fs << "board_height" << board_height;
  fs << "square_size" << square_size;
  cout << "Done Calibration" << endl;

  return 0;
}
