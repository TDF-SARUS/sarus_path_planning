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

# inicializar nodo ros

# Función auxiliar para cambiar un número N de un intervalo (a0, b0) a otro (aF, bF)
def cambiaIntervalo (N, a0, b0, aF, bF):
    return aF + (bF - aF)*(N - a0)/(b0 - a0)

# Para el polígono
def makePolygon (points):
    # Nos dan cuatro puntos
    points = np.array(points)

    # Acotamos el cuadrado
    north = np.amax(points[:,1])
    south = np.amin(points[:,1])
    east = np.amax(points[:,0])
    west = np.amin(points[:,0])

    # Dividimos en grupos de 10
    Ysize = north - south
    Yindexes = round(Ysize/10)          # Número de índices del array_map
    Ywidth = Ysize/Yindexes             # Ancho de barrido que usaremos

    Xsize = north - south
    Xindexes = round(Xsize/10)          # Número de índices del array_map
    Xwidth = Xsize/Xindexes             # Ancho de barrido que usaremos

    y = np.floor(cambiaIntervalo(points[:,1], south, north, 0, Yindexes))
    x = np.floor(cambiaIntervalo(points[:,0], west, east, 0, Xindexes))

    area_maps = polygon(y,x)

    return area_maps, Ywidth, Xwidth

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




