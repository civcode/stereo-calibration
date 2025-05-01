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
  std::string image_name;
  std::string out_file_name;
  int camera_id;
  int image_width;
  int image_height;
  int frames_per_second;
  float scale_factor;

  try {
    cxxopts::Options options(argv[0], "Capture images from two cameras simultaneously");
    options.add_options()
      ("i,cam_id", "Camera ID", cxxopts::value<int>(camera_id)->default_value("0"))
      ("w,img_width", "Image width", cxxopts::value<int>(image_width)->default_value("640"))
      ("h,img_height", "Image height", cxxopts::value<int>(image_height)->default_value("480"))
      ("f,frames_per_second", "Frames per second", cxxopts::value<int>(frames_per_second)->default_value("30"))
      ("s,scale", "Scale factor for image view", cxxopts::value<float>(scale_factor)->default_value("1.0"))
      ("d,img_directory", "Directory to save images in", cxxopts::value<std::string>(image_directory)->default_value("./img"))
      // ("n,img_name", "Image name prefix", cxxopts::value<std::string>(image_name)->default_value("left"))
      ("n,img_name", "Image name prefix", cxxopts::value<std::string>(image_name))
      ("e,extension", "Image extension", cxxopts::value<std::string>(extension)->default_value(".png"))
      ("o,out_file", "Output filename (xml/json/yml)", cxxopts::value<std::string>(out_file_name))
      ("help", "Print help");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      cout << options.help() << endl;
      return 0;
    }

    if (!result.count("img_name")) {
      image_name = "cam-" + std::to_string(camera_id);
    }

    if (result.count("out_file")) {
      out_file_name = result["out_file"].as<std::string>();
    } else {
      out_file_name = image_name + "-img" + ".json";
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

  cout << "Camera ID: " << camera_id << endl;
  cout << "Image settings: " << image_width << "x" << image_height << " @ " << frames_per_second << " FPS" << endl;
  cout << "Display scaling : " << scale_factor << endl;
  cout << "Image directory: " << image_directory << endl;
  cout << "Image name prefix: " << image_name << endl;
  cout << "Image extension: " << extension << endl;
  cout << endl;

  cv::VideoCapture cap(0, cv::CAP_V4L2);

  if (!cap.isOpened()) {
    std::cerr << "Error opening camera" << endl;
    return -1;
  }

  cap.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);
  cap.set(cv::CAP_PROP_FPS, frames_per_second);

  int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  int fps = cap.get(cv::CAP_PROP_FPS);
  cout << "Camera " << camera_id << ": " << width << "x" << height << " @ " << fps << " FPS" << endl;

  cv::Mat img;
  cv::Mat img_res;

  std::vector<std::string> image_names;

  bool is_running = true;
  while (is_running) {
    cap >> img;

    if (img.empty()) {
      std::cerr << "Error capturing image" << endl;
      continue;
    }

    if (scale_factor - 1.0 > 0.01) {
      cv::resize(img, img_res, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);
    } else {
      img_res = img;
    }

    cv::imshow("IMG", img_res);

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
        sprintf(filename, "%s/%s-%.5d%s", image_directory.c_str(), image_name.c_str(), img_count, extension.c_str());
        img_count++;
        bool ret = cv::imwrite(filename, img);
        if (!ret) {
          std::cerr << "Error saving image" << endl;
        } else {
          cout << "Saved image " << img_count << endl;
          image_names.push_back(filename);
        }
        break;
    }
  }

  if (!image_names.empty()) {
    cv::FileStorage fs(out_file_name, cv::FileStorage::WRITE);
    fs << "camera_id" << camera_id;
    fs << "image_width" << image_width;
    fs << "image_height" << image_height;
    fs << "image_names" << image_names;
    fs.release();
    cout << "Saved image names to " << out_file_name << endl;
  }

  cap.release();
  cv::destroyAllWindows();

  return 0;
}