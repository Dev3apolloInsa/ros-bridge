#include <chrono>
#include <math.h>
#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "KDTree-master/KDTree.hpp"
#include "KDTree-master/KDTree.cpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

//Global variable declaration
std::vector<double> veh_pos(3);
double roll, pitch, yaw;
double veh_velocity;
double L = 3.00677;
double Kdd = 4.0;
double alpha_prev = 0;
double delta_prev = 0;
int compt = 0; 
std::vector<double> waypoints_2d;
std::vector<std::vector<double>> waypoint_tree;

class TrajectoryTracking : public rclcpp::Node
{
  public:
    TrajectoryTracking()
    : Node("trajectory_tracking"), count_(0)
    {
      publisher_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&TrajectoryTracking::timer_callback, this));
      
      Odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>("/carla/ego_vehicle/odometry", 10, std::bind(&TrajectoryTracking::odom_callback, this, _1)); 
      
      Vehicule_status = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>("/carla/ego_vehicle/vehicle_status", 10, std::bind(&TrajectoryTracking::Vehicule_status_callback, this, _1)); 
    }

  private: 
    void Vehicule_status_callback(const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msgs) const {
      veh_velocity = msgs->velocity;

      tf2::Quaternion q(msgs->orientation.x, msgs->orientation.y, msgs->orientation.z, msgs->orientation.w);   
      tf2::Matrix3x3 m(q);
      m.getRPY(roll, pitch, yaw);
    } 

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const {
      veh_pos[0] = msg->pose.pose.position.x;
      veh_pos[1] = msg->pose.pose.position.y;
      veh_pos[2] = msg->pose.pose.position.z;
    }
    
    void get_waypoints_list() {
      std::ifstream fichier("/home-local/boubacar/carla-ros2-bridge/src/ros-bridge/carla_trajectory_tracking/src/base_waypoints.txt", std::ios::in);
      if(fichier) {
          std::string line;
          while(getline(fichier, line)) {
              std::istringstream iss(line);
              double x;
              double y;
              iss >> x;
              iss >> y;
              waypoints_2d = {x, y};
              waypoint_tree.push_back(waypoints_2d);    
          }  
          fichier.close(); 
      } else {
          std::cerr << "Impossible d'ouvrir le fichier !" << std::endl;
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
    
    double get_steering_angle(double alpha, double ld) {
      double delta_prev = 0;
      double delta = atan2(2*L*sin(alpha), ld);
      delta = fmax(fmin(delta, 1.0), -1.0);
      if (isnan(delta)) {
          delta = delta_prev;
      }else {
          delta_prev = delta;
      }
      return delta;
    }
  
    void timer_callback()
    {
      auto message = carla_msgs::msg::CarlaEgoVehicleControl();
      if (compt == 0) {
        get_waypoints_list();
        compt = 1;
      }
      KDTree tree(waypoint_tree);
      int closest_idx = tree.nearest_index({veh_pos[0], (-1)*veh_pos[1]});
      double *closest_coord = get_vector(waypoint_tree, closest_idx);
      
      double alpha = atan2(closest_coord[1]+veh_pos[1], closest_coord[0]-veh_pos[0]) + yaw;
      if (std::isnan(alpha)) {
        alpha = alpha_prev;
      } else {
        alpha_prev = alpha;
      }
      
      double ld = Kdd * veh_velocity;
      double steer_angle = get_steering_angle(alpha, ld);

      std::cout << "Vehicle current_position : " << veh_pos[0] << " " << veh_pos[1] <<std::endl;
      std::cout << "Target waypoint index : "<< closest_idx <<std::endl;
      std::cout << "Target Waypoint coord : " <<closest_coord[0] << " " << closest_coord[1] <<std::endl;
      std::cout << "Heading angle : " << alpha <<std::endl;
      std::cout << "Steering angle : " << steer_angle <<std::endl;
      std::cout << "------------------------------------------------------------------"<<std::endl;
      
      message.steer = steer_angle;  
      message.brake = 0;
      if ((closest_idx + 1) == waypoint_tree.size()) {
        message.throttle = 0.0;
      } else {
        message.throttle = 0.2;
      }
      
      publisher_->publish(message); 
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Odom_subscription;
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr Vehicule_status;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryTracking>());
  rclcpp::shutdown();
  return 0;
}
