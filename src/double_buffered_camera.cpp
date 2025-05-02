#include "calibration/double_buffered_camera.hpp"

#include "calibration/types.hpp"

DoubleBufferedCamera::DoubleBufferedCamera(cv::VideoCapture& cap, int id)
    : cap_(cap),
      write_index_(0),
      running_(false),
      id_(id)
{
    count_ = 0;
    buffers_[0].frame = cv::Mat();
    buffers_[1].frame = cv::Mat();
}

DoubleBufferedCamera::~DoubleBufferedCamera() {
    stop();
}

void DoubleBufferedCamera::start() {
    running_ = true;
    capture_thread_ = std::thread(&DoubleBufferedCamera::captureLoop, this);
    std::cout << "Camera " << id_ << " capture thread started" << std::endl;
}
void DoubleBufferedCamera::stop() {
    running_ = false;
    if (capture_thread_.joinable())
        capture_thread_.join();
    // cap_.release();
}
TimestampedFrame DoubleBufferedCamera::getLatestFrame() {
    int index = write_index_.load();
    return buffers_[index];
}

void DoubleBufferedCamera::captureLoop() {

  while (running_) {
      int next_index = 1 - write_index_.load(); // write to inactive buffer
      cap_ >> buffers_[next_index].frame;
      if (buffers_[next_index].frame.empty()) {
          std::cerr << "Failed to capture frame [" << id_ << ", " << count_++ << "]" << std::endl;
          continue;
      }

      if (!buffers_[next_index].frame.empty()) {
            buffers_[next_index].timestamp = std::chrono::steady_clock::now();
          write_index_.store(next_index); // atomic flip
        // std::cout << "Camera [" << id_ << "] captured frame [" << count_++ << "]" << std::endl;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}