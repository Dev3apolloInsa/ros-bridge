##----------------Démarrer CARLA et connecter le client-----------------##
import carla
import random

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
client.set_timeout(10)
world = client.get_world()

map = world.get_map()

spawn_points = world.get_map().get_spawn_points()


# Get the blueprint library and filter for the vehicle blueprints
vehicle = world.get_blueprint_library().filter('*vehicle*')


# Nearest waypoint in the center of a Driving or Sidewalk lane.
waypoint01 = map.get_waypoint(vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))

#Nearest waypoint but specifying OpenDRIVE parameters. 
waypoint02 = map.get_waypoint_xodr(road_id,lane_id,s)

waypoint_list = map.generate_waypoints(2.0)

waypoint_tuple_list = map.get_topology()

my_geolocation = map.transform_to_geolocation(vehicle.transform)


