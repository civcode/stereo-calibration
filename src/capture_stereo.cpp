#include <filesystem>
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cxxopts.hpp"

namespace fs = std::filesystem;

using std::cout;
using std::endl;


int main(int argc, char* argv[])
{
  int img_count = 0;
  std::string image_directory;
  std::string extension;
  std::string out_file_name;
  int image_width;
  int image_height;
  int frames_per_second;
  float scale_factor;

  try {
    cxxopts::Options options(argv[0], "Capture images from two cameras simultaneously");
    options.add_options()
      ("w,img_width", "Image width", cxxopts::value<int>(image_width)->default_value("640"))
      ("h,img_height", "Image height", cxxopts::value<int>(image_height)->default_value("480"))
      ("f,frames_per_second", "Frames per second", cxxopts::value<int>(frames_per_second)->default_value("30"))
      ("s,scale", "Scale factor for displayed image", cxxopts::value<float>(scale_factor)->default_value("1.0"))
      ("d,img_directory", "Directory to save images in", cxxopts::value<std::string>(image_directory)->default_value("./img"))
      ("e,extension", "Image extension", cxxopts::value<std::string>(extension)->default_value(".png"))
      ("o,out_file", "Output filename (xml/json/yml)", cxxopts::value<std::string>(out_file_name))
      ("help", "Print help");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      cout << options.help() << endl;
      return 0;
    }

    if (result.count("out_file")) {
      out_file_name = result["out_file"].as<std::string>();
    } else {
      out_file_name = "stereo-img.json";
    }

  } catch (const cxxopts::OptionException& e) {
    cout << "Error parsing options: " << e.what() << endl;
    return 1;
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

  scale_factor = std::max(0.1f, scale_factor);

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

  cv::Mat img1;
  cv::Mat img_res;
  cv::Mat img2;
  cv::Mat img_res2;
  cv::Mat stereo_image;
  std::vector<std::string> image_names;

  cv::namedWindow("Stereo IMG", cv::WINDOW_AUTOSIZE);
  cv::moveWindow("Stereo IMG", 100, 100);

  bool is_running = true;
  while (is_running) {
    cap1 >> img1;
    cap2 >> img2;

    if (img1.empty() || img2.empty()) {
      std::cerr << "Error capturing images" << endl;
      continue;
    }

    cv::hconcat(img1, img2, stereo_image);

    if (scale_factor - 1.0 > 0.01) {
      cv::resize(stereo_image, img_res, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);
    } else {
      img_res = stereo_image;
    }

    cv::imshow("Stereo IMG", img_res);

    int key = cv::waitKey(1);
    switch (key) {
      case 27: // ESC key
        cout << "ESC key pressed. Exiting..." << endl;
        is_running = false;
        break;
      case 'q':
        cout << "q key pressed. Exiting..." << endl;
        is_running = false;
        break;
      case 's':
        char filename[200];
        sprintf(filename, "%s/stereo-%.5d%s", image_directory.c_str(), img_count, extension.c_str());
        img_count++;

        bool ret = cv::imwrite(filename, stereo_image);
        if (!ret) {
          std::cerr << "Error saving images" << endl;
        } else {
          cout << "Saved stereo image " << img_count << endl;
          image_names.push_back(filename);
        }
        break;
    }
  }

  if (!image_names.empty()) {
    cv::FileStorage fs(out_file_name, cv::FileStorage::WRITE);
    fs << "image_width" << stereo_image.size().width;
    fs << "image_height" << stereo_image.size().height;
    fs << "image_names" << image_names;
    fs.release();
    cout << "Saved image names to " << out_file_name << endl;
  }

  cap1.release();
  cap2.release();
  cv::destroyAllWindows();

  return 0;
}