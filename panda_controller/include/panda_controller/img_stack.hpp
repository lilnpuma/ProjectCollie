/**
 * @file img_stack.hpp
 * @author Manu Madhu Pillai (manump@umd.edu)
 * @brief Header for Vision Stack
 * @version 0.1
 * @date 2022-12-08
 *
 * @copyright Copyright (c) 2022
 * 
 * Copyright [2022] [Akash Ravindra, Manu Madhu Pillai]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef PANDA_CONTROLLER_INCLUDE_PANDA_CONTROLLER_IMG_STACK_HPP_
#define PANDA_CONTROLLER_INCLUDE_PANDA_CONTROLLER_IMG_STACK_HPP_
#include <cv_bridge/cv_bridge.h>

#include <array>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "panda_world/msg/block_location.hpp"
#include "panda_world/msg/multi_block_location.hpp"

namespace vis {
class ImgStack : public rclcpp::Node {
 public:
  /// @brief Constructor for the Walker class
  /// @param name
  ImgStack(std::string name);
  /// @brief Destructor for the Walker class
  ~ImgStack();
  /// @brief Callback function for the image subscriber
  /// @param msg
  void image_callback(const sensor_msgs::msg::Image& msg);
  void get_img();
  cv::Mat thresh_img(cv::Mat image);
  cv::Mat find_contour(cv::Mat image);
  void get_coords(cv::Mat image, std::string color);
  void block_locator(cv::Mat image);

 private:
  /// @brief Coordinate and orientation message
  std::vector<std::array<double, 4>> coord_msg;
  /// @brief Input image
  cv::Mat input_image;
  panda_world::msg::MultiBlockLocation block_locations;
  /// @brief Subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  /// @brief Publisher
  // rclcpp::Publisher<coord_msg>::SharedPtr publisher_; //check the argument
  // std::deque<std::pair<std::string, geometry_msgs::msg::Pose2D>>
  // block_locations;
};
};      // namespace vis
#endif  // PANDA_CONTROLLER_INCLUDE_IMG_STACK_HPP_double
