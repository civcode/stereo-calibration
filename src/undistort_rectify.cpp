#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cxxopts.hpp"

using std::cout;
using std::endl;

int main(int argc, char* argv[])
{
  std::string leftimg_filename;
  std::string rightimg_filename;
  std::string calib_file;
  std::string leftout_filename;
  std::string rightout_filename;

  try {
    cxxopts::Options options("undistort_rectify", "Undistort and rectify stereo images");
    options.add_options()
      ("l,leftimg_filename", "Left image path", cxxopts::value<std::string>(leftimg_filename))
      ("r,rightimg_filename", "Right image path", cxxopts::value<std::string>(rightimg_filename))
      ("c,calib_file", "Stereo calibration file", cxxopts::value<std::string>(calib_file)->default_value("stereo_calib.json"))
      ("o,out_file", "Output calibration filename (YML)", cxxopts::value<std::string>(calib_file)->default_value("stereo_calib.json"))
      ("L,leftout_filename", "Left undistorted image path", cxxopts::value<std::string>(leftout_filename)->default_value("left_undistorted.png"))
      ("R,rightout_filename", "Right undistorted image path", cxxopts::value<std::string>(rightout_filename)->default_value("right_undistorted.png"))
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

  cv::Mat R1, R2, P1, P2, Q;
  cv::Mat K1, K2, R;
  cv::Vec3d T;
  cv::Mat D1, D2;
  cv::Mat img1 = cv::imread(leftimg_filename, cv::IMREAD_COLOR);
  cv::Mat img2 = cv::imread(rightimg_filename, cv::IMREAD_COLOR);

  if (img1.empty() || img2.empty()) {
    cout << "Error reading input images." << endl;
    return 1;
  }

  cv::FileStorage fs1(calib_file, cv::FileStorage::READ);
  if (!fs1.isOpened()) {
    cout << "Error opening calibration file: " << calib_file << endl;
    return 1;
  }

  fs1["K1"] >> K1;
  fs1["K2"] >> K2;
  fs1["D1"] >> D1;
  fs1["D2"] >> D2;
  fs1["R"] >> R;
  fs1["T"] >> T;
  fs1["R1"] >> R1;
  fs1["R2"] >> R2;
  fs1["P1"] >> P1;
  fs1["P2"] >> P2;
  fs1["Q"] >> Q;

  cout << "Left camera matrix: " << K1 << endl;
  cout << "Right camera matrix: " << K2 << endl;
  cout << "Left distortion coefficients: " << D1 << endl;
  cout << "Right distortion coefficients: " << D2 << endl;
  cout << "Rotation matrix: " << R << endl;
  cout << "Translation vector: " << T << endl;
  cout << "Rectification matrix 1: " << R1 << endl;
  cout << "Rectification matrix 2: " << R2 << endl;
  cout << "Projection matrix 1: " << P1 << endl;
  cout << "Projection matrix 2: " << P2 << endl;
  cout << "Disparity-to-depth mapping matrix: " << Q << endl;


  cv::Mat lmapx, lmapy, rmapx, rmapy;
  cv::Mat imgU1, imgU2;

  cv::initUndistortRectifyMap(K1, D1, R1, P1, img1.size(), CV_32F, lmapx, lmapy);
  cv::initUndistortRectifyMap(K2, D2, R2, P2, img2.size(), CV_32F, rmapx, rmapy);
  cv::remap(img1, imgU1, lmapx, lmapy, cv::INTER_LINEAR);
  cv::remap(img2, imgU2, rmapx, rmapy, cv::INTER_LINEAR);

  if (!cv::imwrite(leftout_filename, imgU1)) {
    cout << "Error writing left undistorted image to: " << leftout_filename << endl;
    return 1;
  }

  if (!cv::imwrite(rightout_filename, imgU2)) {
    cout << "Error writing right undistorted image to: " << rightout_filename << endl;
    return 1;
  }

  cv::namedWindow("Left Undistorted Image", cv::WINDOW_NORMAL);
  cv::imshow("Left Undistorted Image", imgU1);
  cv::namedWindow("Right Undistorted Image", cv::WINDOW_NORMAL);
  cv::imshow("Right Undistorted Image", imgU2);
  cv::waitKey(0);

  cout << "Undistortion and rectification completed successfully." << endl;

  return 0;
}
