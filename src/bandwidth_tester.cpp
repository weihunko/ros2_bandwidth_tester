// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <chrono>
#include <vector>
#include "CLI11.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_bandwidth_tester/idl/test_string.hpp"

using namespace std::chrono_literals;

/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */

using MsgString = ros2_bandwidth_tester::idl::TestString;
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  CLI::App app{"ROS2 Bandwidth Tester"};

  std::size_t number_of_publishers = 1;
  std::size_t message_size_in_kb = 1;
  std::uint32_t rate_hz = 100;
  std::uint32_t count = 1000;
  app.add_option("-n,--number", number_of_publishers, "Total number of publishers to be created (default: 1)");
  app.add_option("-s,--size", message_size_in_kb, "Size of the message in kB (default: 1)");
  app.add_option("-r,--rate", rate_hz, "Rate, in hz, for each topic (default: 100)");
  app.add_option("-c,--count", count, "Number of messages to publish before stopping (default: 1000)");

  CLI11_PARSE(app, argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_publisher");
  std::vector<rclcpp::Publisher<MsgString>::SharedPtr> publishers;
  for (std::size_t i = 0; i < number_of_publishers; ++i)
  {
      publishers.push_back(node->create_publisher<MsgString>("topic" + std::to_string(i), rclcpp::QoS(10).reliable()));
  }
  MsgString message;
  message.data = std::string(message_size_in_kb * 1024, 't');
  auto rate_type = 1ms * 1000/rate_hz;
  rclcpp::WallRate loop_rate(rate_type);

  for(uint32_t i = 0; i < count; ++i) {
    RCLCPP_INFO(node->get_logger(), "Publishing %d", i);
    for(auto & publisher : publishers)
    {
        publisher->publish(message);
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
