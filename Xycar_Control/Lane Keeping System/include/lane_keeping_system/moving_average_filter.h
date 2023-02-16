/**
 * @file moving_average_filter.h
 * @author Seungho Hyeong (slkumquat@gmail.com)
 * @brief Moving Average Filter Class header file
 * @version 1.0
 * @date 2023-01-19
 */
#ifndef MOVING_AVERAGE_FILTER_H_
#define MOVING_AVERAGE_FILTER_H_
#include <deque>
#include <iostream>
#include <vector>

namespace xycar {
class MovingAverageFilter final {
public:
  // Construct a new Moving Average Filter object
  MovingAverageFilter(int sample_size);
  // Add new data to filter
  void addSample(int new_sample);
  // Get filtered data
  float getWeightedMovingAverage();
  // Get filtered data
  float getMovingAverage();

private:
  const int kSampleSize_;
  std::deque<int> samples_;
  std::vector<int> weight_;
};
}  // namespace xycar
#endif  // MOVING_AVERAGE_FILTER_H_
