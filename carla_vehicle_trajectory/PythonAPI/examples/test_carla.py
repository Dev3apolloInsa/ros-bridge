##----------------Démarrer CARLA et connecter le client-----------------##
import carla
import random

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
world = client.get_world()

##----------------Charger une carte alternative ou recharger la carte actuelle-----------------##

# Print available maps
client.get_available_maps()

# Load new map
#client.load_world('Town07')

# Reload current map and reset state
#client.reload_world()

##----------------L'objet world-----------------##

# Get names of all objects 
world.get_names_of_all_objects()
#print(world.get_names_of_all_objects())

# Filter the list of names for buildings
#filter(lambda x: 'Building' in x, world.get_names_of_all_objects())

# Get a list of all actors, such as vehicles and pedestrians
world.get_actors()

##----------------Ajout d'un vehicule dans un emplacement aléatoire-----------------##

# Get the blueprint library and filter for the vehicle blueprints
#vehicle_bps = world.get_blueprint_library().filter('*vehicle*')

# Randomly choose a vehicle blueprint to spawn
#vehicle_bp = random.choice(vehicle_bps)
#print(vehicle_bp)

# We need a place to spawn the vehicle that will work so we will
# use the predefined spawn points for the map and randomly select one
#spawn_point = random.choice(world.get_map().get_spawn_points())
#print(world.get_map().get_spawn_points())

# Now let's spawn the vehicle
#world.spawn_actor(vehicle_bp, spawn_point)

#For various reasons, this spawn attempt might fail, so to avoid our code crashing, we can use a fault tolerant spawn method.
#vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

# Filter the list to find the vehicles
#print(world.get_actors().filter('*vehicle*'))

##----------------Le spectateur et ses propriétés-----------------##
# Retrieve the spectator object
#spectator = world.get_spectator()

# Get the location and rotation of the spectator through its transform
#transform = spectator.get_transform()

#location = transform.location
#rotation = transform.rotation
#print(location)
#print(rotation)

#vehicle = world.try_spawn_actor(vehicle_bp, spectator.get_transform())

#print(spectator.get_transform())

# Set the spectator with an empty transform
#spectator.set_transform(carla.Transform())
# This will set the spectator at the origin of the map, with 0 degrees
# pitch, yaw and roll - a good way to orient yourself in the map

##----------------visualisation des points d'apparition de la carte-----------------##

# Get the map spawn points
spawn_points = world.get_map().get_spawn_points()

for i, spawn_point in enumerate(spawn_points):
    # Draw in the spectator window the spawn point index
    world.debug.draw_string(spawn_point.location, str(i), life_time=100)
    # We can also draw an arrow to see the orientation of the spawn point
    # (i.e. which way the vehicle will be facing when spawned)
    world.debug.draw_arrow(spawn_point.location, spawn_point.location + spawn_point.get_forward_vector(), life_time=100)
    if i == 117: 
        print(spawn_point.location)

##----------------Actors and blueprints-----------------##

# Print all available vehicle blueprints
#for actor in world.get_blueprint_library().filter('vehicle'):
#    print(actor)

#vehicle_blueprint = world.get_blueprint_library().find('vehicle.audi.tt')
#print(vehicle_blueprint)

##----------------

#waypoints = world.map.generate_waypoints(1.0)
#for w in waypoints:
#    world.world.debug.draw_string(w.transform.location, 'O', draw_shadow=False,
#                                       color=carla.Color(r=255, g=0, b=0), life_time=120.0,
#                                       persistent_lines=True)


