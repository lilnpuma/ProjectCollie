/**
 * @file motion_stack.cpp
 * @author Akash Ravindra (aravind2@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-12-14
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
#include "motion_stack.hpp"

namespace pc_moveit {
MotionStack::MotionStack(std::string name)
    : Node(name),
      m_arm_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), ARM_GROUP),
      m_gripper_group_(std::shared_ptr<rclcpp::Node>(std::move(this)),
                       HAND_GROUP) {
    // Subscribe to the block location topic
  m_block_locations_sub_ =
      this->create_subscription<panda_world::msg::MultiBlockLocation>(
          "/block_info", 10,
          std::bind(&MotionStack::blockLocationCallback, this,
                    std::placeholders::_1));
    m_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&MotionStack::performPickPlace, this));
      m_timer_->reset();
}
MotionStack::~MotionStack() {}
void MotionStack::blockLocationCallback(
    const panda_world::msg::MultiBlockLocation& msg) {
    m_timer_->cancel();
  RCLCPP_INFO(this->get_logger(), "Received block info");
  // Store all the block locations
  for (auto block : msg.blocks) {
    // Skip the block if it is not visible
    if (block.color.length() == 0) {
      continue;
    }
    // Store the block color and pose
    auto block_info = std::make_pair(block.color, block.pose);
    m_block_locations_.push_back(block_info);
  }
  m_timer_->reset();
}

geometry_msgs::msg::Pose MotionStack::convertPose2DToPose(
    geometry_msgs::msg::Pose2D pose2d, float z = 0.0) {
      // This function is used to convert a 2d pose to a 3d pose with a given z offset
  geometry_msgs::msg::Pose pose;
  pose.position.x = pose2d.x;
  pose.position.y = pose2d.y;
  pose.position.z = z;
  tf2::Quaternion q;
  // Set the orientation of the pose
  q.setRPY(3.141, 0.0, pose2d.theta);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}
geometry_msgs::msg::Pose MotionStack::getPlacePose(std::string& block_name) {
  // Place pose depends on the color of the block picked
  geometry_msgs::msg::Pose place_pose;
  // Place pose is always above the bin
  place_pose.position.z = PLACE_HEIGHT;
  if (block_name == "red") {
    // Place pose is to the left of the bin
    place_pose.position.x = -0.5;
    place_pose.position.y = 0.5;
  } else if (block_name == "blue") {
    // Place pose is to the center of the bin
    place_pose.position.x = 0.0;
    place_pose.position.y = 0.5;
  } else if (block_name == "green") {
    // Place pose is to the right of the bin
    place_pose.position.x = 0.5;
    place_pose.position.y = 0.5;
  } else {
    // Drop block
    place_pose.position.x = 0.5;
    place_pose.position.y = 0;
  }
  return place_pose;
}
geometry_msgs::msg::Pose MotionStack::getPickPose(
    geometry_msgs::msg::Pose2D block_pose, bool pre_grasp = false) {
      // Pick pose is either above the block or at the block
  return convertPose2DToPose(block_pose,
                             pre_grasp ? PRE_GRASP_HEIGHT : PICK_HEIGHT);
}
void MotionStack::moveToPose(geometry_msgs::msg::Pose pose) {
  // Create a pose stamped message from the pose
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "panda_link0";
  pose_stamped.pose = pose;
  // Move to the pose
  m_arm_group_.setPoseTarget(pose);
  // Block until the motion is complete
  m_arm_group_.move();
}
void MotionStack::moveToPose(std::string pose_name) {
  // Move to the named pose
  m_arm_group_.setNamedTarget(pose_name);
  // Block until the motion is complete
  m_arm_group_.move();
}
void MotionStack::moveGripper(bool open = true) {
  m_gripper_group_.setNamedTarget(open ? "open" : "close");
  m_gripper_group_.move();
}
void MotionStack::performPickPlace() {
  while(!m_block_locations_.empty()) {
    // Stop the timer
    m_timer_->cancel();
    auto block = m_block_locations_.front();
    m_block_locations_.pop_front();
    RCLCPP_INFO(this->get_logger(), "Picking block %s", block.first.c_str());
    // Pick
    moveToPose("ready");
    moveToPose(getPickPose(block.second, true));
    moveGripper(true);
    moveToPose(getPickPose(block.second));
    moveGripper(false);
    RCLCPP_INFO(this->get_logger(), "Finished Picking block %s", block.first.c_str());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    // Place
    moveToPose("ready");
    RCLCPP_INFO(this->get_logger(), "Placing block %s", block.first.c_str());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    moveToPose(getPlacePose(block.first));
    moveGripper(true);
    moveGripper(false);
  }
  // Restart the timer
  m_timer_->reset();
}
}  // namespace pc_moveit