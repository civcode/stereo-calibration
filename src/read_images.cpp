#include <filesystem>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cxxopts.hpp"

namespace fs = std::filesystem;

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
  float scale_factor;

  cxxopts::Options options(argv[0], "Capture images from two cameras simultaneously");
  options.add_options()
    ("w,img_width", "Image width", cxxopts::value<int>(image_width)->default_value("640"))
    ("h,img_height", "Image height", cxxopts::value<int>(image_height)->default_value("480"))
    ("f,frames_per_second", "Frames per second", cxxopts::value<int>(frames_per_second)->default_value("30"))
    ("s,scale", "Scale factor for displayed image", cxxopts::value<float>(scale_factor)->default_value("1.0"))
    ("d,img_directory", "Directory to save images in", cxxopts::value<std::string>(image_directory)->default_value("./img"))
    ("e,extension", "Image extension", cxxopts::value<std::string>(extension)->default_value(".png"))
    ("help", "Print help");

  auto result = options.parse(argc, argv);

  if (result.count("help")) {
    cout << options.help() << endl;
    return 0;
  }

  try {
    if (!fs::exists(image_directory)) {
      if (fs::create_directories(image_directory)) {
        cout << "Directory created: " << image_directory << endl;
      } else {
        std::cerr << "Error creating directory: " << image_directory << endl;
        return -1;
      }
    }
  } catch (const fs::filesystem_error& e) {
    std::cerr << "Filesystem error: " << e.what() << endl;
    return -1;
  }

  cout << "Image settings: " << image_width << "x" << image_height << " @ " << frames_per_second << " FPS" << endl;
  cout << "Display scaling : " << scale_factor << endl;
  cout << "Image directory: " << image_directory << endl;
  cout << "Image extension: " << extension << endl;
  cout << endl;

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

    cv::resize(img1, img_res1, cv::Size(image_width*scale_factor, image_height*scale_factor));
    cv::resize(img2, img_res2, cv::Size(image_width*scale_factor, image_height*scale_factor));

    cv::imshow("IMG1", img_res1);
    cv::imshow("IMG2", img_res2);

    int key = cv::waitKey(1);
    switch (key) {
      case 27: // ESC key
        cout << "ESC key pressed. Exiting..." << endl;
        return 0;
      case 'q':
        cout << "q key pressed. Exiting..." << endl;
        return 0;
      case 's':
        char filename1[200], filename2[200];
        sprintf(filename1, "%s/left%d%s", image_directory.c_str(), x, extension.c_str());
        sprintf(filename2, "%s/right%d%s", image_directory.c_str(), x, extension.c_str());
        x++;
        // cout << "filename1: " << filename1 << endl;
        // cout << "Saving img pair " << x << endl;
        bool ret1 = cv::imwrite(filename1, img_res1);
        bool ret2 = cv::imwrite(filename2, img_res2);
        if (!ret1 || !ret2) {
          std::cerr << "Error saving images" << endl;
        } else {
          cout << "Saved image pair " << x << endl;
        }
        break;
    }
  }

  return 0;
}