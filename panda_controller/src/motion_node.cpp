/**
 * @file motion_node.cpp
 * @author Akash Ravindra (aravind2@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-12-13
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

#include <rclcpp/rclcpp.hpp>

#include "motion_stack.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto motion_stack =
      std::make_shared<pc_moveit::MotionStack>("panda_controller_motion");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(motion_stack);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
