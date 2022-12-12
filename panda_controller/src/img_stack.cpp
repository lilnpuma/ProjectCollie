/**
 * @file img_stack.cpp
 * @author Manu Madhu Pillai (manump@umd.edu)
 * @brief Contains Vision Stack Implementation
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
 */

#include "img_stack.hpp"

#include <cmath>

#include "geometry_msgs/msg/pose2_d.hpp"

vis::ImgStack::ImgStack(std::string name) : Node(name) {
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", 10,
      std::bind(&ImgStack::image_callback, this, std::placeholders::_1));
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  cv::namedWindow("view");
}

vis::ImgStack::~ImgStack() { cv::destroyWindow("view"); }

void vis::ImgStack::image_callback(const sensor_msgs::msg::Image &msg) {
  vis::ImgStack::input_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
  cv::imshow("view", vis::ImgStack::input_image);
  cv::waitKey(3);
}

void vis::ImgStack::get_img() {}

cv::Mat vis::ImgStack::thresh_img(cv::Mat image) {
  cv::Mat color_img;
  cv::threshold(image, color_img, 180, 255, cv::THRESH_BINARY);
  cv::imshow("view", color_img);
  cv::waitKey(3);
  return color_img;
}

cv::Mat vis::ImgStack::find_contour(cv::Mat image) {
  cv::Mat edges, blur_img;
  /// Blur image
  cv::GaussianBlur(image, blur_img, cv::Size(3, 3), 0);
  /// Detect edges using canny
  cv::Canny(blur_img, edges, 100, 200, 3, false);
  cv::imshow("view", edges);
  cv::waitKey(3);
  return edges;
}

void vis::ImgStack::get_coords(cv::Mat image, std::string color) {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  /// Get Contours
  cv::findContours(image, contours, hierarchy, cv::RETR_TREE,
                   cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  double scaling_factor;
  scaling_factor = 0.2 / cv::arcLength(contours[1], true);
  /// Get the moments
  int contours_size = (int)contours.size();
  std::vector<cv::Moments> mu(contours.size());
  for (int i = 0; i < contours_size; i++) {
    mu[i] = cv::moments(contours[i], false);
  }
  ///  Get the centroid of figures and orientation
  std::vector<std::vector<cv::Point2f>> rect(contours.size());
  std::vector<panda_world::msg::BlockLocation> block_loc;

  for (int i = 0; i < contours_size; i++) {
    cv::boxPoints(cv::minAreaRect(contours[i]), rect[i]);
    geometry_msgs::msg::Pose2D new_pose;
    new_pose.x = (mu[i].m10 / mu[i].m00 - image.size[0] / 2) * scaling_factor;
    new_pose.y = (mu[i].m01 / mu[i].m00 - image.size[1] / 2) * scaling_factor;

    if (rect[i][1].x == rect[i][0].x) {
      new_pose.theta = 0.0;
    } else {
      new_pose.theta =
          atan2((rect[i][1].y - rect[i][0].y), (rect[i][1].x - rect[i][0].x));
    }
    panda_world::msg::BlockLocation new_loc;
    new_loc.color = color;
    new_loc.pose = new_pose;

    vis::ImgStack::block_locations.blocks.push_back(new_loc);
  }
}

void vis::ImgStack::block_locator(cv::Mat image) {
  cv::Mat color_img[3], thres_img;
  cv::split(image, color_img);
  std::vector<std::string> colors;
  colors.push_back("blue");
  colors.push_back("green");
  colors.push_back("red");
  std::vector<panda_world::msg::MultiBlockLocation> block_loc(3);
  for (int i = 0; i < 3; i++) {
    color_img[i] = vis::ImgStack::thresh_img(color_img[i]);
    color_img[i] = vis::ImgStack::find_contour(color_img[i]);
    vis::ImgStack::get_coords(color_img[i], colors[i]);
  }
}
