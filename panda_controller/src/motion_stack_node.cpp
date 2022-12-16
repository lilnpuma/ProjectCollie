/**
 * @file motion_stack_node.cpp
 * @author Manu Madhu Pillai (manump@umd.edu)
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
#include "motion_stack_node.hpp"

namespace pc_moveit
{
  MotionStackNode::MotionStackNode(std::string name)
    : Node(name),
    m_arm_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), ARM_GROUP),
    m_gripper_group_(std::shared_ptr<rclcpp::Node>(std::move(this)),
                     HAND_GROUP)
  {
    // Subscribe to the block location topic
    m_motion_stack_.m_block_locations_sub_ =
      this->create_subscription<panda_world::msg::MultiBlockLocation>(
        "/block_info", 10,
        std::bind(&MotionStackNode::blockLocationCallback, this,
                  std::placeholders::_1));
    m_motion_stack_.m_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&MotionStackNode::performPickPlace, this));
    m_motion_stack_.m_timer_->reset();
  }

  void MotionStackNode::performPickPlace()
  {
    while (!m_motion_stack_.m_block_locations_.empty())
    {
      // Stop the timer
      m_motion_stack_.m_timer_->cancel();
      auto block = m_motion_stack_.m_block_locations_.front();
      m_motion_stack_.m_block_locations_.pop_front();
      RCLCPP_INFO(this->get_logger(), "Picking block %s", block.first.c_str());

      // Pick
      moveToPose("ready");
      moveToPose(m_motion_stack_.getPickPose(block.second, true));
      moveGripper(true);
      moveToPose(m_motion_stack_.getPickPose(block.second, false));
      moveGripper(false);
      RCLCPP_INFO(this->get_logger(), "Finished Picking block %s",
                  block.first.c_str());
      rclcpp::sleep_for(std::chrono::milliseconds(100));

      // Place
      moveToPose("ready");
      RCLCPP_INFO(this->get_logger(), "Placing block %s", block.first.c_str());
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      moveToPose(m_motion_stack_.getPlacePose(block.first));
      moveGripper(true);
      moveGripper(false);
    }

    // Restart the timer
    m_motion_stack_.m_timer_->reset();
  }

  void MotionStackNode::blockLocationCallback(
    const panda_world::msg::MultiBlockLocation& msg)
  {
    m_motion_stack_.m_timer_->cancel();

    // Store all the block locations
    for (auto block : msg.blocks)
    {
      // Skip the block if it is not visible
      if (block.color.length() == 0)
      {
        continue;
      }

      // Store the block color and pose
      auto block_info = std::make_pair(block.color, block.pose);
      m_motion_stack_.m_block_locations_.push_back(block_info);
    }
    m_motion_stack_.m_timer_->reset();
  }

  void MotionStackNode::moveToPose(geometry_msgs::msg::Pose pose)
  {
    // Create a pose stamped message from the pose
    geometry_msgs::msg::PoseStamped pose_stamped;

    pose_stamped.header.frame_id = "panda_link0";
    pose_stamped.pose            = pose;

    // Move to the pose
    m_arm_group_.setPoseTarget(pose);

    // Block until the motion is complete
    m_arm_group_.move();
  }

  void MotionStackNode::moveToPose(std::string pose_name)
  {
    // Move to the named pose
    m_arm_group_.setNamedTarget(pose_name);

    // Block until the motion is complete
    m_arm_group_.move();
  }

  void MotionStackNode::moveGripper(bool open = true)
  {
    m_gripper_group_.setNamedTarget(open ? "open" : "close");
    m_gripper_group_.move();
  }
} // namespace pc_moveit
