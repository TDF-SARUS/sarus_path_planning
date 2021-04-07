#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import matplotlib.pyplot as plt
from cpp_algorithms import dist_fill
from cpp_algorithms import darp, stc, bcd, wavefront
from cpp_algorithms import get_drone_map, get_random_coords
from cpp_algorithms import get_all_area_maps, plot, imshow, imshow_scatter
from cpp_algorithms.darp.darp_helpers import get_assigned_count
from cpp_algorithms.coverage_path.pathing_helpers import has_isolated_areas


# In[2]:


n = 3
area_maps = get_all_area_maps("test_maps")
area_map = area_maps[0]
start_points = get_random_coords(area_map, n)


# In[3]:


start_points = get_random_coords(area_map, n)
A, losses = darp(300, area_map, start_points, pbar=True)
drone_maps = [get_drone_map(A,i) for i in range(n)]




coverage_paths = [bcd(drone_maps[i],start_points[i]) for i in range(n)]





plt.show()
