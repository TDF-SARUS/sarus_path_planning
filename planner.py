#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  ADD DESCRIPTION
#
#
#


from __future__ import division
from __future__ import absolute_import
import numpy as np
import matplotlib.pyplot as plt
import rospy
import std_msgs.msg
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseStamped
#from skimage.draw import polygon

from cpp_algorithms import darp, stc, bcd, wavefront
from cpp_algorithms import get_drone_map, get_random_coords
from cpp_algorithms import get_all_area_maps, plot, imshow, imshow_scatter
from cpp_algorithms.darp.darp_helpers import get_assigned_count
from cpp_algorithms.coverage_path.pathing_helpers import has_isolated_areas


# Change coordinates
def cambiaIntervalo (N, a0, b0, aF, bF):
    return aF + (bF - aF)*(N - a0)/(b0 - a0)

def gazebo2alg (point):
    newPoint = [[tuple([cambiaIntervalo(coord, -100, 100, 0, 31) for coord in drone]) for drone in drones] for drones in point]
    return newPoint

def alg2gazebo (point):
    newPoint = [[tuple([cambiaIntervalo(coord, 0, 31, -100, 100) for coord in drone]) for drone in drones] for drones in point]
    return newPoint

def makePolygon (points, width=10):	#Polygon to bitmap with pixels of ~ 10x10 m

    points = np.array(points)

    # Acotamos el cuadrado
    north = np.amax(points[:,1])
    south = np.amin(points[:,1])
    east = np.amax(points[:,0])
    west = np.amin(points[:,0])

    # Dividimos en grupos de 10
    Ysize = north - south
    Yindexes = round(Ysize/width)	# Numero de indices del array_map
    Ywidth = Ysize/Yindexes             # Ancho de barrido que usaremos

    Xsize = north - south
    Xindexes = round(Xsize/10)          # Numero de indices del array_map
    Xwidth = Xsize/Xindexes             # Ancho de barrido que usaremos

    y = np.floor(cambiaIntervalo(points[:,1], south, north, 0, Yindexes))
    x = np.floor(cambiaIntervalo(points[:,0], west, east, 0, Xindexes))

#    area_maps = polygon(y,x)
    area_maps = 1

    return area_maps, Ywidth, Xwidth

def initPublisher(droneNum):
    # Define the name of the drone's topic
    topic=u'/drone11'+unicode(droneNum)+u'/motion_reference/pose'
    # Define the publisher and type of message
    pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
    return pub

def publishTrajectory(pub, coordinates):
    msg = PoseStamped()
    # Also possible with numpy
    #[x_ref, y_ref] = np.split(np.array(coordinates), 2, axis = 1)
    # Separate the position in its coordinates
    msg.pose.position.x=coordinates[0]
    msg.pose.position.y=coordinates[1]
    msg.pose.position.z=1.0 #coordinates[2]

    # Publish the message (coordinates of the position of drone droneNum)
    pub.publish(msg)

def poly_cb(data):
    global base_polygon
    base_polygon=np.array(data)

def n_cb(data):
    global n
    n=data



########################
### Program Start ######
########################


rospy.init_node(u'coords_node', anonymous=True) #Create node

# Frequency of the sleep
rate = rospy.Rate(0.2) #0.2 Hz -> 5s

# Datos de Interfaz:

n=3   #n de drones

base_polygon = [] #poligono del mapa



rospy.Subscriber(u'/interfaz/poligono', INT32MultiArray, poly_cb)
rospy.Subscriber(u'/interfaz/poligono', Int8, poly_cb)


area_maps = get_all_area_maps(u'test_maps')
area_map = area_maps[1]

#area_map, Xwidth, Ywidth=makePolygon(pol) #Make bitmap from polygon

start_points = get_random_coords(area_map, n) # Random start coordinates

A, losses = darp(300, area_map, start_points, pbar=True) # Area division algorithm

drone_maps = [get_drone_map(A,i) for i in xrange(n)] #assign a map for each drone

coverage_paths = [bcd(drone_maps[i],start_points[i]) for i in xrange(n)]  #Calculate the routes for each drone



# sacar un msg/path para interfaz
########### FALTA POR HACER ##########



#################################################
### meter topic salida a aerostack drone111/#####
#################################################

coverage_path_gazebo = alg2gazebo(coverage_paths)


# Drone with the maximum number of positions: Each drone has a different path with different number of points
maxPos=0
for drone in xrange(len(coverage_path_gazebo)):        # Number of drones
    numberPos = len(coverage_path_gazebo[drone])
    if numberPos>maxPos:
        maxPos=numberPos

pubs = []
for drone in xrange(n):        # Number of drones
    pubs.append(initPublisher(drone+1)) #+1 because in Python first array is 0


# Execute the function for every position and for all of the drones
for pos in xrange(maxPos):                                # Maximum number of positions
    for drone in xrange(len(coverage_path_gazebo)):        # Number of drones
        # Try and except to deal with the problem of having different number of positions
        try:
            # Call the function publishTrajectory sending the number of the drone and the desired position
            publishTrajectory(pubs[drone], coverage_path_gazebo[drone][pos])
            print u'Drone ',drone+1, u' = ', coverage_path_gazebo[drone][pos]
        except:
            print u'Drone ',drone+1, u' has reached its final position'
    print
    rate.sleep() # 5s



