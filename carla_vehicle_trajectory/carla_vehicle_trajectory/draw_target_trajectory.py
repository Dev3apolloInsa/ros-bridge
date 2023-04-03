# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import carla
import sys
import time
import os
import csv
sys.path.append('../')
from agents.navigation.global_route_planner import GlobalRoutePlanner


class Vehicle_trajectory(Node):

    def __init__(self):
        super().__init__('vehicle_trajectory')
        self.publisher_ = self.create_publisher(String, '/carla/ego_vehicle/trajectory', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.j = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Traget trajectory: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        
        
        if self.j % 10 == 0:
            # Connectez-vous au client et récupérez l'objet World
            client = carla.Client("localhost", 2000)
            client.set_timeout(10)
            world = client.get_world()
   
            # Demander au serveur le XODR contenant le fichier de carte et le renvoie analysé comme carla.Map.
            carla_map = world.get_map()
        
            # variable qui détermine la distance entre les points de cheminements
            sampling_resolution = 2.0

            # Initialiser le plan de route
            grp = GlobalRoutePlanner(carla_map, sampling_resolution)
    
            # Définir le point d'origine et le point d'arriver 
            point_of_origin = carla.Location(x=20.0, y=2.5, z=0.5)
            target_point = carla.Location(x=1.0, y=2.5, z=0.5)
    
            # Cette méthode renvoie la liste des (carla.Waypoint, RoadOption) de l'origine à la destination
            w1 = grp.trace_route(point_of_origin, target_point)
           
            for w in w1:    
            #world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=True,
            #color = carla.Color(r=0, g=0, b=255), life_time=1,
            #persistent_lines=True)
        
                world.debug.draw_point(w[0].transform.location, size =0.07, color = carla.Color(r=255, g=0, b=0),
                life_time=5, persistent_lines=True)
        self.j += 1
        

def main(args=None):
    rclpy.init(args=args)

    vehicle_trajectory = Vehicle_trajectory()

    rclpy.spin(vehicle_trajectory)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vehicle_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
