#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "cxxopts.hpp"

using std::cout;
using std::endl;

int x = 0;

int main(int argc, char* argv[])
{
  std::string image_directory;
  std::string extension;
  int image_width;
  int image_height;
  int frames_per_second;

  cxxopts::Options options(argv[0], "Capture images from two cameras simultaneously");
  options.add_options()
    ("w,img_width", "Image width", cxxopts::value<int>(image_width)->default_value("640"))
    ("h,img_height", "Image height", cxxopts::value<int>(image_height)->default_value("480"))
    ("d,img_directory", "Directory to save images in", cxxopts::value<std::string>(image_directory)->default_value("./"))
    ("f,frames_per_second", "Frames per second", cxxopts::value<int>(frames_per_second)->default_value("30"))
    ("e,extension", "Image extension", cxxopts::value<std::string>(extension)->default_value("png"))
    ("help", "Print help");

  auto result = options.parse(argc, argv);


  cv::VideoCapture cap1(0, cv::CAP_V4L2);
  cv::VideoCapture cap2(1, cv::CAP_V4L2);

  if (!cap1.isOpened() || !cap2.isOpened()) {
    std::cerr << "Error opening cameras" << endl;
    return -1;
  }

  cap1.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
  cap1.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);
  cap1.set(cv::CAP_PROP_FPS, frames_per_second);

  cap2.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
  cap2.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);
  cap2.set(cv::CAP_PROP_FPS, frames_per_second);

  int width = cap1.get(cv::CAP_PROP_FRAME_WIDTH);
  int height = cap1.get(cv::CAP_PROP_FRAME_HEIGHT);
  int fps = cap1.get(cv::CAP_PROP_FPS);
  cout << "Camera 1: " << width << "x" << height << " @ " << fps << " FPS" << endl;

  width = cap2.get(cv::CAP_PROP_FRAME_WIDTH);
  height = cap2.get(cv::CAP_PROP_FRAME_HEIGHT);
  fps = cap2.get(cv::CAP_PROP_FPS);
  cout << "Camera 2: " << width << "x" << height << " @ " << fps << " FPS" << endl;

  cv::Mat img1, img_res1, img2, img_res2;

  while (true) {
    cap1 >> img1;
    cap2 >> img2;

    if (img1.empty() || img2.empty()) {
      std::cerr << "Error capturing images" << endl;
      continue;
    }

    cv::resize(img1, img_res1, cv::Size(image_width, image_height));
    cv::resize(img2, img_res2, cv::Size(image_width, image_height));

    cv::imshow("IMG1", img_res1);
    cv::imshow("IMG2", img_res2);

    if (cv::waitKey(1) > 0) {
      x++;
      char filename1[200], filename2[200];
      sprintf(filename1, "%sleft%d.%s", image_directory.c_str(), x, extension.c_str());
      sprintf(filename2, "%sright%d.%s", image_directory.c_str(), x, extension.c_str());
      cout << "Saving img pair " << x << endl;
      cv::imwrite(filename1, img_res1);
      cv::imwrite(filename2, img_res2);
    }
  }

  return 0;
}