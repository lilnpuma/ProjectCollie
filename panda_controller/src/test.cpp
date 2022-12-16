/**
 * @file test.cpp
 * @author your name (you@domain.com)
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
#include <panda_world/msg/block_location.hpp>
#include <panda_world/msg/multi_block_location.hpp>
#include <rclcpp/rclcpp.hpp>

class simpleChecker : public rclcpp::Node {
public:

  simpleChecker() : Node("simple_checker")
  {
    pub_ = this->create_publisher<panda_world::msg::MultiBlockLocation>(
      "block_info", 10);
    panda_world::msg::BlockLocation block1;

    block1.color      = "red";
    block1.pose.x     = -0.45;
    block1.pose.y     = -0.2;
    block1.pose.theta = -0.3;
    panda_world::msg::BlockLocation block2;

    block2.color      = "green";
    block2.pose.x     = 0.45;
    block2.pose.y     = -0.5;
    block2.pose.theta = -0.6;
    panda_world::msg::MultiBlockLocation msg;

    msg.blocks.push_back(block1);
    msg.blocks.push_back(block2);
    pub_->publish(msg);
  }

private:

  rclcpp::Publisher<panda_world::msg::MultiBlockLocation>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::make_shared<simpleChecker>();
  rclcpp::shutdown();
  return 0;
}
