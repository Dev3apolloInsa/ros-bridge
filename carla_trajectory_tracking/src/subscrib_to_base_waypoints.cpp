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

#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "carla_msgs/msg/lane.hpp"
#include "carla_msgs/msg/waypoint.hpp"
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<carla_msgs::msg::Lane>(
      "/base_waypoint", 100, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const carla_msgs::msg::Lane::SharedPtr msg) const
  {
    auto message = carla_msgs::msg::Lane();
    message.waypoints = msg->waypoints;
    for (size_t i=0; i<message.waypoints.size(); i++) {
        std::cout << "Position: "<< message.waypoints[i].pose.pose.position.x << "  " << message.waypoints[i].pose.pose.position.y << "  " << message.waypoints[i].pose.pose.position.z << std::endl;
    } 
    
  }
  rclcpp::Subscription<carla_msgs::msg::Lane>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
