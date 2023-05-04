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
#include <unistd.h> 
#include "rclcpp/rclcpp.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include <iostream>
#include <fstream>
#include <string>

using namespace std::chrono_literals;

//Globale variables
int read_cmpt = 0;
int lane = 1; 
std::vector<double> target_x;
std::vector<double> target_y;
std::vector<double> target_z;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Tracking_controller : public rclcpp::Node
{
public:
  Tracking_controller()
  : Node("tracking_controller"), count_(0)
  {
    publisher_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&Tracking_controller::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if(read_cmpt == 0)
    {
        std::cout << "début de la boucle" << std::endl;
        std::ifstream fichier("/home-local/boubacar/carla-ros2-bridge/src/ros-bridge/carla_trajectory_tracking/src/out.txt", std::ios::in);
        if(fichier) 
        {
            std::string ligne;
            while(getline(fichier, ligne)) {
                std::istringstream iss(ligne);
                double x;
                double y;
                double z;
                iss >> x;
                iss >> y;
                iss >> z;
                target_x.push_back(x);
                target_y.push_back(y);
                target_z.push_back(z);
                //std::cout << ligne << std::endl;
            }    
            fichier.close();
        } else {
            std::cerr << "Impossible d'ouvrir le fichier !" << std::endl;
        }

        //vérification
        /*for(int i = 0; i < target_x.size(); i++){
            std::cout << target_x[i] << std::endl;
        }*/
        
        std::cout << "fin de la boucle" << std::endl;
        read_cmpt = 1;
    }
    
    
    auto message = carla_msgs::msg::CarlaEgoVehicleControl(); 
    message.steer = 1;  
    message.brake = 0;  
    message.throttle = 0.1;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{ 
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tracking_controller>());
  rclcpp::shutdown();
  return 0;
}
