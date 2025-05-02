#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <chrono>

#include <opencv2/core/core.hpp>

struct TimestampedFrame {
    cv::Mat frame;
    std::chrono::steady_clock::time_point timestamp;
};

struct VisualizationConfig {
  bool visualize;
  int wait_time;
};


#endif // TYPES_HPP_