#!/usr/bin/env python



import numpy as np
import matplotlib.pyplot as plt
from cpp_algorithms import dist_fill
from cpp_algorithms import darp, stc, bcd, wavefront
from cpp_algorithms import get_drone_map, get_random_coords
from cpp_algorithms import get_all_area_maps, plot, imshow, imshow_scatter
from cpp_algorithms.darp.darp_helpers import get_assigned_count
from cpp_algorithms.coverage_path.pathing_helpers import has_isolated_areas
from skimage.draw import polygon
from 

# inicializar nodo ros

# Para el polígono
def polygon (points):
    # Nos dan cuatro puntos
    points = np.array(points)

    # Acotamos el cuadrado
    north = np.amax(points[:,1])
    south = np.amin(points[:,1])
    east = np.amax(points[:,0])
    west = np.amin(points[:,0])

    # Dividimos en grupos de 10
    tam = north - south
    tam2 = round(tam/10)    # Número de índices del array_map
    tam3 = tam/tam2         # Ancho de barrido que usaremos

    x = 

    polygon(y,x)

    return tam3

n = 3
area_maps = get_all_area_maps("test_maps")
area_map = area_maps[0]
start_points = get_random_coords(area_map, n)


start_points = get_random_coords(area_map, n)
A, losses = darp(300, area_map, start_points, pbar=True)
drone_maps = [get_drone_map(A,i) for i in range(n)]


coverage_paths = [bcd(drone_maps[i],start_points[i]) for i in range(n)]

# meter topic salida a aerostack drone111/



# sacar un msg/path para interfaz




