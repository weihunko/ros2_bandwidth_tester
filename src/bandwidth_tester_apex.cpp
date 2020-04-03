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
#include "std_msgs/msg/u_int64_multi_array.hpp"

using namespace std::chrono_literals;

/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  CLI::App app{"ROS2 Bandwidth Tester"};

  std::size_t number_of_publishers = 1;
  std::size_t message_size_in_kb = 1;
  std::uint32_t rate_hz = 100;
  std::uint32_t count = 1000;
  app.add_option("-n,--number", number_of_publishers, "Total number of publishers to be created (default: 1)");
  app.add_option("-s,--size", message_size_in_kb, "Size of the message in kB (default: 1, max 512)");
  app.add_option("-r,--rate", rate_hz, "Rate, in hz, for each topic (default: 100)");
  app.add_option("-c,--count", count, "Number of messages to publish before stopping (default: 1000)");

  CLI11_PARSE(app, argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_publisher");
  std::vector<rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr> publishers;
  for (std::size_t i = 0; i < number_of_publishers; ++i)
  {
      publishers.push_back(node->create_publisher<std_msgs::msg::UInt64MultiArray>("topic" + std::to_string(i), rclcpp::QoS(10).reliable()));
  }
  std_msgs::msg::UInt64MultiArray message;
  /* message.data = std::string(message_size_in_kb * 1024, 't'); */
  std_msgs::msg::MultiArrayDimension dim0;
  dim0.label="123"; // make this 4 bytes (3 + null) so that the layout portion of the message is exactly 16 bytes
  dim0.size=65536;
  dim0.stride = 1;
  message.layout.dim.push_back(dim0);
  message.layout.data_offset = 0;
  auto num_elements = message_size_in_kb * 1024 / 8 - 2; // subtract 2 since the layout takes up 16 bytes
  message.data.assign(num_elements, 0);
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
