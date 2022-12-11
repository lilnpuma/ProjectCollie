/**
 * @file motion_stack.hpp
 * @author Akash Ravindra (aravind2@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-12-13
 *
 * MIT License
 * Copyright (c) 2022 Akash Ravindra
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef PANDA_CONTROLLER_INCLUDE_PANDA_CONTROLLER_MOTION_STACK_HPP_
#define PANDA_CONTROLLER_INCLUDE_PANDA_CONTROLLER_MOTION_STACK_HPP_
#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <deque>

#include "panda_world/msg/block_location.hpp"
#include "panda_world/msg/multi_block_location.hpp"
#include "tf2/LinearMath/Quaternion.h"

/// @brief Name of the Panda robot arm move group
constexpr char ARM_GROUP[]{"panda_arm"};
/// @brief Name of the panda robot gripper move group
constexpr char HAND_GROUP[]{"panda_gripper"};
/// @brief The Z offset of the robot base frame from the world frame
constexpr double ROBOT_BASE_FRAME_HEIGHT = 1.0;
/// @brief The Z offset of the table from the robot base
constexpr double TABLE_HEIGHT = ROBOT_BASE_FRAME_HEIGHT - 1.0;
/// @brief The Z offset of the camera from the robot base
constexpr double CAMERA_HEIGHT = ROBOT_BASE_FRAME_HEIGHT - 2.75;
/// @brief The height of each block
constexpr double BLOCK_HEIGHT = 0.05;
/// @brief The height of each bin
constexpr double BIN_HEIGHT = 0.2;
/// @brief The Z offset of the pre-grasp pose from the robot base
constexpr double PRE_GRASP_HEIGHT = TABLE_HEIGHT + BLOCK_HEIGHT * 5.0;
/// @brief The Z offset of the pick pose from the robot base
constexpr double PICK_HEIGHT = TABLE_HEIGHT + BLOCK_HEIGHT;
/// @brief The Z offset of the place pose from the robot base
constexpr double PLACE_HEIGHT = TABLE_HEIGHT + BIN_HEIGHT * 1.5;

namespace pc_moveit {
class MotionStack : public rclcpp::Node {
 public:
  /// @brief Constructor that initializes the move groups and subscribes to the
  /// block location topic
  /// @param name
  MotionStack(std::string name);
  /// @brief Destructor
  ~MotionStack();
  /// @brief Used to move the robot arm to a given pose
  /// @param goal_pose
  void moveToPose(geometry_msgs::msg::Pose goal_pose);
  /// @brief Used to move the robot arm to a given pose name
  /// @param pose_name 
  void moveToPose(std::string pose_name);
  /// @brief Used to move the robot gripper to a given pose
  /// @param open true to open the gripper, false to close the gripper
  void moveGripper(bool open);
  /// @brief Used to get the pick pose of a given block pose
  /// @param block_pose x, y and theta of the block
  /// @param pre_grasp true to get the pre-grasp pose, false to get the pick
  /// pose
  /// @return full 3d pose of the block
  geometry_msgs::msg::Pose getPickPose(geometry_msgs::msg::Pose2D block_pose,
                                       bool pre_grasp);
  /// @brief Used to get the place pose of a given block color
  /// @param block_color color of the block
  /// @return Full 3d pose of the bin placement
  geometry_msgs::msg::Pose getPlacePose(std::string& block_color);
  /// @brief The main function that performs the pick and place operation by
  /// calling the other functions systematically in the correct order
  void performPickPlace();

 private:
  /// @brief Converts a 2d pose to a 3d pose with a given z offset
  /// @param pose2d x, y and theta of the pose
  /// @param z z offset of the pose
  /// @return full 3d pose
  geometry_msgs::msg::Pose convertPose2DToPose(
      geometry_msgs::msg::Pose2D pose2d, float z);
  /// @brief The arm moveit interface group, the main interface to the robot arm
  moveit::planning_interface::MoveGroupInterface m_arm_group_;
  /// @brief The gripper moveit interface group, the main interface to the robot gripper
  moveit::planning_interface::MoveGroupInterface m_gripper_group_;
  /// @brief The block location subscriber
  rclcpp::Subscription<panda_world::msg::MultiBlockLocation>::SharedPtr
      m_block_locations_sub_;
  /// @brief The block location callback
  /// @param msg
  void blockLocationCallback(const panda_world::msg::MultiBlockLocation& msg);
  /// @brief The block locations
  std::deque<std::pair<std::string, geometry_msgs::msg::Pose2D>>
      m_block_locations_;
    /// @brief The timer used to call the pick and place function
    rclcpp::TimerBase::SharedPtr m_timer_;
};
}  // namespace pc_moveit

#endif /* PANDA_CONTROLLER_INCLUDE_PANDA_CONTROLLER_MOTION_STACK_HPP_ */
