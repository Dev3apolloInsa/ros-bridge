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

#include <chrono>
#include <memory>
#include <fstream>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "carla_msgs/msg/lane.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "carla_msgs/msg/waypoint.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "rclcpp/qos.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

double velocity;
double MAX_DECEL = 1.0;
std::vector<carla_msgs::msg::Waypoint> waypoints_;

class Waypoint_processing : public rclcpp::Node
{
public:
  Waypoint_processing()
  : Node("Waypoint_processing"), count_(0)
  {
    Odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
      "/carla/ego_vehicle/odometry", 10, std::bind(&Waypoint_processing::odom_callback, this, _1)); 
      
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);
    publisher_2 = this->create_publisher<carla_msgs::msg::Lane>("/base_waypoint", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    new_waypoint_loader(); 
  } 

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const {
    auto message = geometry_msgs::msg::PoseStamped();
    message.header.stamp = msg->header.stamp;
    message.header.frame_id = "/world";
    message.pose.position.x = msg->pose.pose.position.x;
    message.pose.position.y = msg->pose.pose.position.y;
    message.pose.position.z = msg->pose.pose.position.z;
    message.pose.orientation.x = msg->pose.pose.orientation.x;
    message.pose.orientation.y = msg->pose.pose.orientation.y;
    message.pose.orientation.z = msg->pose.pose.orientation.z;
    message.pose.orientation.w = msg->pose.pose.orientation.w;
    publisher_->publish(message);
  } 
  
  double distance(double px, double py, double pz, double qx, double qy, double qz) {
    double x = px - qx;
    double y = py - qy;
    double z = pz - qz;
    return sqrt(x*x + y*y + z*z);
  }

  void new_waypoint_loader() {
    //Velocity kmph2mps
    velocity = (40 * 1000.) / (60. * 60.);
    std::ifstream fichier("/home-local/boubacar/carla-ros2-bridge/src/ros-bridge/carla_trajectory_tracking/src/base_waypoints.txt", std::ios::in);
    if(fichier) {
        std::string ligne;
        while(getline(fichier, ligne)) {
            std::istringstream iss(ligne);
            carla_msgs::msg::Waypoint p;
            tf2::Quaternion q;
            double x;
            double y;
            double z;
            double w;
            iss >> x;
            iss >> y;
            iss >> z;
            iss >> w;
            //Affectation de la position
            p.pose.pose.position.x = x;
            p.pose.pose.position.y = y;
            p.pose.pose.position.z = z;
            //Convertion des rotations cibles roulis, de tangage et de lacet en un quaternion.
            q.setRPY(0., 0., w);
            p.pose.pose.orientation.x = q.x();
            p.pose.pose.orientation.y = q.y();
            p.pose.pose.orientation.z = q.z();
            p.pose.pose.orientation.w = q.w();
            //Affectation de la vitesse cible entre les waypoints
            p.twist.twist.linear.x = velocity;
            waypoints_.push_back(p);     
        }  
        fichier.close(); 
    } else {
        std::cerr << "Impossible d'ouvrir le fichier !" << std::endl;
    }
    
    carla_msgs::msg::Waypoint last;
    last = waypoints_.back();
    last.twist.twist.linear.x = 0.;
    for (auto wp = waypoints_.rbegin(); wp != waypoints_.rend() - 1; ++wp) {
        double dist = distance(wp->pose.pose.position.x, wp->pose.pose.position.y, 
                                wp->pose.pose.position.z, last.pose.pose.position.x, 
                                last.pose.pose.position.y, last.pose.pose.position.z);
        double vel = sqrt(2 * MAX_DECEL * dist);
        if (vel < 1.) {
            vel = 0.;
        }
        wp->twist.twist.linear.x = std::min(vel, wp->twist.twist.linear.x);
    }  
    
    auto lane = carla_msgs::msg::Lane();
    lane.header.frame_id = "/world";
    //lane.header.stamp = this->get_clock()->now();
    lane.waypoints = waypoints_; 
    publisher_2->publish(lane);
    
    // Pour les vérifications des valeurs
    /*for (size_t i=0; i<lane.waypoints.size(); i++) {
        //std::cout << lane.waypoints[i].twist.twist.linear.x << std::endl;
        std::cout << lane.waypoints[i].pose.pose.orientation.w << std::endl;
    }*/ 
  }
  
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<carla_msgs::msg::Lane>::SharedPtr publisher_2;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Odom_subscription;
  size_t count_;
};

int main(int argc, char * argv[])
{ 
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Waypoint_processing>());
  rclcpp::shutdown();
  return 0;
}
