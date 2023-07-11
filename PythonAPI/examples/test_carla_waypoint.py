import carla
import sys
import time
import os
import csv
sys.path.append('../')
from agents.navigation.global_route_planner import GlobalRoutePlanner

# Connectez-vous au client et récupérez l'objet World
client = carla.Client("localhost", 2000)
client.set_timeout(10)

# Charger une nouvelle carte ou Recharger la carte actuelle et réinitialiser l'état
#world = client.load_world('Town05')
#world = client.get_available_maps()

world = client.get_world()

# Demander au serveur le XODR contenant le fichier de carte et le renvoie analysé comme carla.Map.
carla_map = world.get_map()

# variable qui détermine la distance entre les points de cheminements
sampling_resolution = 1.5

# Initialiser le plan de route
grp = GlobalRoutePlanner(carla_map, sampling_resolution)

# Obtenir les points d'apparition de la carte
spawn_points = world.get_map().get_spawn_points()
#print(spawn_points)

# Définir le point d'origine et le point d'arriver 
#point_of_origin = carla.Location(x=20.0, y=2.5, z=0.5)
#target_point = carla.Location(x=2.0, y=2.5, z=0.5)

# Définir le point d'origine et le point d'arriver 
point_of_origin = carla.Location(x=20.0, y=2.5, z=0.5)
target_point = carla.Location(x=1.0, y=2.5, z=0.5)

# Cette méthode renvoie la liste des (carla.Waypoint, RoadOption) de l'origine à la destination
w1 = grp.trace_route(point_of_origin, target_point)
#print(w1)

# le repertoire de travail actuel
#print(os.getcwd())

# Dessiner les points d'apparutions sur la map (Pb: visible que coté serveur)
#i = 0   
#for w in w1:
#    lt = 60
#    if i % 10 == 0:
#        world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
#        color=carla.Color(r=255, g=0, b=0), life_time=120.0,
#        persistent_lines=True)
        
#        world.debug.draw_point(w[0].transform.location, life_time=lt,
#        persistent_lines=False)      
        
        #world.debug.draw_point(w[0].transform.location, life_time=lt, life_time=120.0, persistent_lines=True)
#    else:
#        world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
#        color = carla.Color(r=0, g=0, b=255), life_time=120.0,
#        persistent_lines=True)
        
#        world.debug.draw_point(w[0].transform.location, life_time=lt,
#        persistent_lines=False)
#    i += 1

# Dessiner et enregistrer les points d'apparutions dans un fichier .txt
#i = 0
#with open("out.txt", "a") as fichier:
#    for w in w1:
#        if i % 10 == 0:
#            world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
#            color=carla.Color(r=255, g=0, b=0), life_time=60.0,
#            persistent_lines=True)
#        else:
#            world.debug.draw_string(w[0].transform.location, 'o', draw_shadow=False,
#            color = carla.Color(r=0, g=0, b=255), life_time=60.0,
#            persistent_lines=True) 
#        # Stockoge des coordonnées de la boucle 
#        fichier.writelines(str(w[0].transform.location) + '\n')
#        #time.sleep(1)
#        i += 1
#fichier.close()  


# Dessiner et enregistrer les points d'apparutions dans un fichier csv
#i = 0
#with open('data_csv.csv', "w") as fichier:
#    for w in w1:
#        if i % 10 == 0:
#            world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
#            color=carla.Color(r=255, g=0, b=0), life_time=60.0,
#            persistent_lines=True)
#        else:
#            world.debug.draw_string(w[0].transform.location, 'o', draw_shadow=False,
#            color = carla.Color(r=0, g=0, b=255), life_time=60.0,
#            persistent_lines=True) 
        # Stockoge des coordonnées de la boucle 
#        writer = csv.writer(fichier)
#        writer.writerow(str(w[0].transform.location) + '\n')
        #time.sleep(1)
#        i += 1
#fichier.close() 


i = 0   
for w in w1:    
    world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=True,
    color = carla.Color(r=0, g=0, b=255), life_time=60.0,
    persistent_lines=True)
        
    world.debug.draw_point(w[0].transform.location, size =0.07, color = carla.Color(r=255, g=0, b=0),
    life_time=60, persistent_lines=False)
    i += 1


