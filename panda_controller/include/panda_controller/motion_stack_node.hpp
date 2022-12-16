/**
 * @file motion_stack_node.hpp
 * @author Akash Ravindra (aravind2@umd.edu)
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
 *
 */
#ifndef PANDA_CONTROLLER_INCLUDE_PANDA_CONTROLLER_MOTION_STACK_NODE_HPP_
#define PANDA_CONTROLLER_INCLUDE_PANDA_CONTROLLER_MOTION_STACK_NODE_HPP_
#include <moveit/move_group_interface/move_group_interface.h>

#include <deque>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include "motion_stack.hpp"
#include "panda_world/msg/block_location.hpp"
#include "panda_world/msg/multi_block_location.hpp"
#include "tf2/LinearMath/Quaternion.h"

/// @brief Name of the Panda robot arm move group
constexpr char ARM_GROUP[]{ "panda_arm" };

/// @brief Name of the panda robot gripper move group
constexpr char HAND_GROUP[]{ "panda_gripper" };

namespace pc_moveit
{
  class MotionStackNode : public rclcpp::Node {
public:

    MotionStackNode(std::string name);
    ~MotionStackNode()
    {}

    moveit::planning_interface::MoveGroupInterface& get_arm_group()
    {
      return m_arm_group_;
    }

    moveit::planning_interface::MoveGroupInterface& get_gripper_group()
    {
      return m_gripper_group_;
    }

    void performPickPlace();

    /// @brief Used to move the robot arm to a given pose
    /// @param goal_pose
    void moveToPose(geometry_msgs::msg::Pose goal_pose);

    /// @brief Used to move the robot arm to a given pose name
    /// @param pose_name
    void moveToPose(std::string pose_name);

    /// @brief Used to move the robot gripper to a given pose
    /// @param open true to open the gripper, false to close the gripper
    void moveGripper(bool open);

    /// @brief The block location callback
    /// @param msg
    void blockLocationCallback(const panda_world::msg::MultiBlockLocation& msg);
    MotionStack m_motion_stack_;

private:

    /// @brief The arm moveit interface group, the main interface to the robot
    // arm
    moveit::planning_interface::MoveGroupInterface m_arm_group_;

    /// @brief The gripper moveit interface group, the main interface to the
    // robot
    /// gripper
    moveit::planning_interface::MoveGroupInterface m_gripper_group_;
  };
} // namespace pc_moveit
#endif // ifndef
       // PANDA_CONTROLLER_INCLUDE_PANDA_CONTROLLER_MOTION_STACK_NODE_HPP_
