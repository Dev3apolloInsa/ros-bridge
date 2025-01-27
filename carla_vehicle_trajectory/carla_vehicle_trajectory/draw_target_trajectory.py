import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import glob
import sys
import time
import carla

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
        
            # Connect to the client and retrieve the world object
            client = carla.Client("localhost", 2000)
            client.set_timeout(10)
            world = client.get_world()
   
            # Ask the server for the XODR containing the map file and returns it parsed as carla.Map.
            carla_map = world.get_map()
        
            # Variable that determines the distance between waypoints
            sampling_resolution = 2.0

            # Initialize the route plan
            grp = GlobalRoutePlanner(carla_map, sampling_resolution)
    
            # Define the starting point and the ending point
            point_of_origin = carla.Location(x=20.0, y=2.5, z=0.5)
            target_point = carla.Location(x=1.0, y=2.5, z=0.5)
    
            # This method returns the list of (carla.Waypoint, RoadOption) from origin to destination
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
