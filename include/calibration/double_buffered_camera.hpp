#ifndef DOUBLE_BUFFERED_CAMERA_HPP_
#define DOUBLE_BUFFERED_CAMERA_HPP_

#include <opencv2/opencv.hpp>
#include <atomic>
#include <chrono>
#include <thread>
#include <array>
#include <memory>

#include <iostream>
#include <cstdlib>

#include "calibration/types.hpp"

class DoubleBufferedCamera {
public:
    explicit DoubleBufferedCamera(cv::VideoCapture& cap, int id = 0);
    ~DoubleBufferedCamera();

    void start();
    void stop();
    TimestampedFrame getLatestFrame();

private:
    cv::VideoCapture& cap_;
    std::atomic<int> write_index_; // 0 or 1
    std::atomic<bool> running_;
    std::array<TimestampedFrame, 2> buffers_;
    std::thread capture_thread_;
    int count_;
    int id_;

    void captureLoop();
};

#endif // DOUBLE_BUFFERED_CAMERA_HPP_