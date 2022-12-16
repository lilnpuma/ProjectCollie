/**
 * @file test.cpp
 * @author Manu Madhu Pillai (manump@terpmail.umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-12-15
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

// #include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <stdlib.h>

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>

#include "../include/panda_controller/img_stack.hpp"
#include "../include/panda_controller/motion_stack.hpp"

const std::string mstck = "motion_stack";

// Level 1 Test Definitions (gtest)

TEST(MotionTest, convertPose2DToPose)
{
  pc_moveit::MotionStack motion_stack;
  std::vector<double>    init_pose = { 0.0, 10.5, 62.4 };
  geometry_msgs::msg::Pose2D goal_pose_2d;

  goal_pose_2d.x     = 0.0;
  goal_pose_2d.y     = 0.3;
  goal_pose_2d.theta = 1.57;
  float z = 0;
  geometry_msgs::msg::Pose goal_pose;

  goal_pose.position.x = 0.0;
  goal_pose.position.y = 0.3;
  goal_pose.position.z = 0.0;
  tf2::Quaternion q;

  // Set the orientation of the pose
  q.setRPY(3.141, 0.0, 1.57);
  goal_pose.orientation.x = q.x();
  goal_pose.orientation.y = q.y();
  goal_pose.orientation.z = q.z();
  goal_pose.orientation.w = q.w();

  ASSERT_EQ(motion_stack.convertPose2DToPose(goal_pose_2d, z), goal_pose);
}

TEST(MotionTest, getPlacePose)
{
  pc_moveit::MotionStack motion_stack;

  // std::vector<double> init_pose = { 0.0, 10.5, 62.4 };
  geometry_msgs::msg::Pose2D goal_pose_2d;
  std::string block_name = "red";

  goal_pose_2d.x     = 0.0;
  goal_pose_2d.y     = 0.3;
  goal_pose_2d.theta = 1.57;

  geometry_msgs::msg::Pose goal_pose;

  goal_pose.position.x = -0.5;
  goal_pose.position.y = 0.5;
  goal_pose.position.z = PLACE_HEIGHT;

  tf2::Quaternion q;


  ASSERT_EQ(motion_stack.getPlacePose(block_name), goal_pose);
}

TEST(MotionTest, getPickPose)
{
  pc_moveit::MotionStack motion_stack;
  std::vector<double>    init_pose = { 0.0, 10.5, 62.4 };
  geometry_msgs::msg::Pose2D goal_pose_2d;
  bool pre_grasp{ true };

  goal_pose_2d.x     = 0.0;
  goal_pose_2d.y     = 0.3;
  goal_pose_2d.theta = 1.57;

  // float z = 1.2;
  geometry_msgs::msg::Pose goal_pose;

  goal_pose.position.x = 0.0;
  goal_pose.position.y = 0.3;
  goal_pose.position.z = PRE_GRASP_HEIGHT;
  tf2::Quaternion q;

  // Set the orientation of the pose
  q.setRPY(3.141, 0.0, 1.57);
  goal_pose.orientation.x = q.x();
  goal_pose.orientation.y = q.y();
  goal_pose.orientation.z = q.z();
  goal_pose.orientation.w = q.w();

  ASSERT_EQ(motion_stack.getPickPose(goal_pose_2d, pre_grasp), goal_pose);
}


TEST(ImgTest, thresh_img)
{
  std::string   img_stack_name = "img";
  vis::ImgStack img_stack(img_stack_name);
  cv::Mat op_img;
  cv::Mat ip_img(8, 8, CV_8UC3, cv::Scalar(0, 0, 255));

  cv::threshold(ip_img, op_img, 180, 255, cv::THRESH_BINARY);


  ASSERT_EQ(!cv::norm(img_stack.thresh_img(ip_img), op_img, cv::NORM_L1), true);
}


TEST(ImgTest, find_contour)
{
  std::string   img_stack_name = "img";
  vis::ImgStack img_stack(img_stack_name);
  cv::Mat op_img;
  cv::Mat ip_img(16, 16, CV_8UC3, cv::Scalar(0, 0, 255));

  cv::threshold(ip_img, op_img, 180, 255, cv::THRESH_BINARY);
  cv::Mat edges, blur_img;

  cv::GaussianBlur(op_img, blur_img, cv::Size(3, 3), 0);
  cv::Canny(blur_img, edges, 100, 200, 3, false);


  ASSERT_EQ(!cv::norm(img_stack.find_contour(op_img), edges, cv::NORM_L1), true);
}

TEST(WorldTest, Blockloc)
{
  panda_world::msg::BlockLocation new_block;

  new_block.color = "red";
  std::string color { "red" };

  ASSERT_EQ(new_block.color, color);
}


TEST(ImgTest, block_locator)
{
  std::string   img_stack_name = "img";
  vis::ImgStack img_stack(img_stack_name);
  std::string   color { "red" };
  cv::Mat op_img;
  cv::Mat ip_img(32, 32, CV_8UC3, cv::Scalar(0, 0, 255));

  cv::threshold(ip_img, op_img, 180, 255, cv::THRESH_BINARY);
  cv::Mat edges, blur_img;

  cv::GaussianBlur(op_img, blur_img, cv::Size(3, 3), 0);
  cv::Canny(blur_img, edges, 100, 200, 3, false);
  img_stack.block_locator(ip_img);


  ASSERT_EQ(img_stack.block_color, color);
}
