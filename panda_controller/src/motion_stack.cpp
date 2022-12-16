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

namespace pc_moveit
{
  MotionStack::MotionStack()
  {}

  MotionStack::~MotionStack()
  {}

  geometry_msgs::msg::Pose MotionStack::convertPose2DToPose(
    geometry_msgs::msg::Pose2D pose2d, float z = 0.0)
  {
    // This function is used to convert a 2d pose to a 3d pose with a given z
    // offset
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

  geometry_msgs::msg::Pose MotionStack::getPlacePose(std::string& block_name)
  {
    // Place pose depends on the color of the block picked
    geometry_msgs::msg::Pose place_pose;

    // Place pose is always above the bin
    place_pose.position.z = PLACE_HEIGHT;

    if (block_name == "red")
    {
      // Place pose is to the left of the bin
      place_pose.position.x = -0.5;
      place_pose.position.y = 0.5;
    }
    else if (block_name == "blue")
    {
      // Place pose is to the center of the bin
      place_pose.position.x = 0.0;
      place_pose.position.y = 0.5;
    }
    else if (block_name == "green")
    {
      // Place pose is to the right of the bin
      place_pose.position.x = 0.5;
      place_pose.position.y = 0.5;
    }
    else
    {
      // Drop block
      place_pose.position.x = 0.5;
      place_pose.position.y = 0;
    }
    return place_pose;
  }

  geometry_msgs::msg::Pose MotionStack::getPickPose(
    geometry_msgs::msg::Pose2D block_pose, bool pre_grasp = false)
  {
    // Pick pose is either above the block or at the block
    return convertPose2DToPose(block_pose,
                               pre_grasp ? PRE_GRASP_HEIGHT : PICK_HEIGHT);
  }
} // namespace pc_moveit
