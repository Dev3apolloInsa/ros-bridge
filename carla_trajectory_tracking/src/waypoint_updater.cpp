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
#include <vector>
#include <iostream>
#include <functional> // std::plus
#include <algorithm>  // std::transform
#include "rclcpp/rclcpp.hpp"
#include "KDTree-master/KDTree.hpp"
#include "KDTree-master/KDTree.cpp"
#include "std_msgs/msg/string.hpp"
#include "carla_msgs/msg/lane.hpp"
#include "carla_msgs/msg/waypoint.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <unistd.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

int LOOKAHEAD_WPS = 10;
double MAX_DECEL = 0.5;
int stopline_wp_idx = -1;

carla_msgs::msg::Lane final_lane;
geometry_msgs::msg::PoseStamped pose;
carla_msgs::msg::Lane base_lane;
carla_msgs::msg::Lane base_waypoints;
std::vector<double> waypoints_2d;
std::vector<std::vector<double>> waypoint_tree;


class WaypointUpdater : public rclcpp::Node
{
public:
  WaypointUpdater()
  : Node("waypoint_updater")
  {
    subscription_2 = this->create_subscription<carla_msgs::msg::Lane>(
      "/base_waypoint", 10, std::bind(&WaypointUpdater::waypoints_cb, this, _1));
    subscription_1 = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/current_pose", 10, std::bind(&WaypointUpdater::pose_cb, this, _1));
  
    final_waypoints_pub = this->create_publisher<carla_msgs::msg::Lane>("final_waypoints", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&WaypointUpdater::publish_waypoints, this));    
  }

private:

  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pose = *msg;
    std::cout << "test pose_cb" << std::endl;
  }
  
  void waypoints_cb(const carla_msgs::msg::Lane::SharedPtr msg) { 
    //base_lane.header.stamp = msg->header.stamp;
    base_lane.waypoints = msg->waypoints;
    std::cout << "test waypoint_cb" << std::endl;
    if (waypoints_2d.empty()) {
        for (auto waypoint : base_lane.waypoints) {
            waypoints_2d = {waypoint.pose.pose.position.x, waypoint.pose.pose.position.y};
            waypoint_tree.push_back(waypoints_2d);
        }
    }
  } 
  
  double *get_vector(std::vector<std::vector<double>> waypoints, int index) {
    double *point = new double[2];
    for(int i = 0; i < waypoint_tree.size(); i++) {
        if ( i == index){
            for(int j = 0; j < waypoint_tree[i].size(); j++)
                point[j] = waypoint_tree[i][j];  
        }
    }
    return point;
  }
  
  double dot_product(double *v1, double *v2){
    double product = 0;
    for(int i=0;i<5;i++)
        product += v1[i] * v2[i];
    return product;
  } 
  
  int get_closest_waypoint_idx() {
    double x = pose.pose.position.x;
    double y = (-1) * pose.pose.position.y;
    //std::cout << x << " " << y << std::endl;
    
    KDTree tree(waypoint_tree);
    int closest_idx = tree.nearest_index({x, y});
    std::cout << closest_idx <<std::endl;
    
    double *closest_coord = get_vector(waypoint_tree, closest_idx);
    double *prev_coord = get_vector(waypoint_tree, closest_idx - 1);
    double pos_vect[2] = {x, y};
    double cl_vect[2]; 
    double pv_vect[2];
    std::transform(closest_coord, closest_coord + 2, prev_coord, cl_vect, std::minus<double>());
    std::transform(pos_vect, pos_vect + 2, closest_coord, pv_vect, std::minus<double>());
    
    double val = dot_product(cl_vect, pv_vect); 
    
    if (val > 0) {
        closest_idx = (closest_idx + 1) % waypoint_tree.size();
    }
    return closest_idx;
  }
  
  //Fonction pour l'extraction d'un sous-vecteur
  template<typename T>
  std::vector<T> slice(std::vector<T> const &v, int m, int n)
  {
    auto first = v.cbegin() + m;
    auto last = v.cbegin() + n + 1;
 
    std::vector<T> vec(first, last);
    return vec;
  }
  
  /*carla_msgs::msg::Waypoint decelerate_waypoints(carla_msgs::msg::Waypoint waypoints, int closest_idx) {
    carla_msgs::msg::Waypoint temp;
    for (int i = 0; i < waypoints.size(); i++) {
        carla_msgs::msg::Waypoint p;
        p.pose = waypoints[i].pose;
        
        int stop_idx = std::max(stopline_wp_idx - closest_idx - 3, 0);
        double dist = distance(waypoints, i, stop_idx);
        double vel = sqrt(2*MAX_DECEL*dist);
        
        if (vel < 0.5) {  // 1.0 
                vel = 0.;
            }
            p.twist.twist.linear.x = std::min(vel, waypoints[i].twist.twist.linear.x);
            temp.push_back(p);
        }
        return temp;
  }*/
  
  /*double distance(carla_msgs::msg::Waypoint waypoints, int wp1, int wp2){
	double dist = 0;
	auto dl = [](auto a, auto b) {
		return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2) + pow((a.z - b.z), 2));
	};
	for (int i = wp1; i <= wp2; i++)
	{
		dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position);
		wp1 = i;
	}
	return dist;
  }*/
  
  
  carla_msgs::msg::Lane generate_lane() {
    carla_msgs::msg::Lane lane;
    int closest_idx = get_closest_waypoint_idx();
    int farthest_idx = closest_idx + LOOKAHEAD_WPS;
    if(base_lane.waypoints.size() != 0){
        base_waypoints.waypoints = slice(base_lane.waypoints, closest_idx, farthest_idx);
        if (stopline_wp_idx == -1 || (stopline_wp_idx >= farthest_idx)) {
            lane.waypoints = base_waypoints.waypoints;
        }
        /*else {
            lane.waypoints = decelerate_waypoints(base_waypoints.waypoints, closest_idx);
        }*/
    }
    return lane;
  }

  void publish_waypoints(){
    std::cout << "test" << std::endl;
    //final_lane.header.stamp = this->get_clock()->now();
    final_lane.header = base_lane.header;
    final_lane = generate_lane();
    final_waypoints_pub->publish(final_lane);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_1;
  rclcpp::Subscription<carla_msgs::msg::Lane>::SharedPtr subscription_2;
  rclcpp::Publisher<carla_msgs::msg::Lane>::SharedPtr final_waypoints_pub;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointUpdater>());
  rclcpp::shutdown();
  return 0;
}
