#!/usr/bin/env python3
#
#  ADD DESCRIPTION
#
#
#

import numpy as np
import matplotlib.pyplot as plt

import rospy
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from skimage.draw import polygon

from cpp_algorithms import darp, stc, bcd, wavefront
from cpp_algorithms import get_drone_map, get_random_coords
from cpp_algorithms import get_all_area_maps, plot, imshow, imshow_scatter
from cpp_algorithms.darp.darp_helpers import get_assigned_count
from cpp_algorithms.coverage_path.pathing_helpers import has_isolated_areas
from cpp_algorithms import dist_fill


def cambiaIntervalo (N, a0, b0, aF, bF):
    return aF + (bF - aF)*(N - a0)/(b0 - a0)

def alg2gazebo (point, Yindexes, Xindexes, north, south, east, west):
    newPoint = [[[cambiaIntervalo(drone[0], 0, Xindexes-1, west, east), cambiaIntervalo(drone[1], 0, Yindexes-1, south, north), 15] for drone in drones] for drones in point]
    return newPoint
'''
def gazebo2alg (point):
    newPoint = [[[cambiaIntervalo(coord, -100, 100, 0, 31) for coord in drone] for drone in drones] for drones in point]
    return newPoint
'''
def makePolygon (points, width=10):	#Polygon to bitmap with pixels of ~ 10x10 m
    # Acotamos el cuadrado
    north = points[0].y
    south = points[0].y
    east = points[0].x
    west = points[0].x
    for point in points:
        if point.y > north:
            north = point.y
        if point.y < south:
            south = point.y
        if point.x > east:
            east = point.x
        if point.x < west:
            west = point.x
    # Dividimos en grupos de 10
    Ysize = north - south
    Yindexes = np.floor(Ysize/width)		#N of divisions
    Yindexes = np.int8(Yindexes)
    Ywidth = Ysize/Yindexes			#exact width of divisions
    Xsize = north - south
    Xindexes = np.floor(Xsize/width)
    Xindexes = np.int8(Xindexes)
    Xwidth = Xsize/Xindexes
    y = [];
    x = [];
    for point in points:
        y.append(np.floor(cambiaIntervalo(point.y, south, north, 0, Yindexes)))
        x.append(np.floor(cambiaIntervalo(point.x, west, east, 0, Xindexes)))

    print("Square width: " + str(Ysize)+"x"+str(Xsize))
    print("N of squares: " + str(Yindexes)+"x"+str(Xindexes))
    map_x, map_y = polygon(y,x)
    area_map =np.zeros((Yindexes, Xindexes), dtype=np.int8)
    area_map[map_y, map_x]=1
    area_map=area_map-1
    return area_map, Yindexes, Xindexes, north, south, east, west

def appendPath(coordinates, to_C2):
    msg = PoseStamped()
    # Also possible with numpy
    # Separate the position in its coordinates
    msg.pose.position.x=coordinates[0]
    msg.pose.position.y=coordinates[1]
    if to_C2 == False:
        msg.pose.position.z=15.0 #coordinates[2]
    msg.header.frame_id = 'world'
    return msg

def poly_cb(data):
    global base_polygon
    base_polygon=np.array(data.polygon.points)
    rospy.loginfo('Polygon received')
    global update
    update = True


def n_cb(data):
    global n
    n=data



########################
### Program Start ######
########################
rospy.init_node('coords_node', anonymous=True) #Create node

pubC2 = rospy.Publisher('/mapviz/path', Path, queue_size=10)
pubAerostack = rospy.Publisher('topic', Path, queue_size=10)

# Frequency of the sleep
rate = rospy.Rate(0.2) #0.2 Hz -> 5s
rospy.loginfo('Mision Planner ready')

# Datos de Interfaz:

n=1   #n de drones FALTA QUE NOS LO MANDE JAVI

old_polygon = np.empty(2)

#rospy.Subscriber('/interfaz/poligono', Int32MultiArray, poly_cb)
rospy.Subscriber('/mapviz/polygon', PolygonStamped, poly_cb)


while True:
    if 'update' in vars() and update == 1:
        rospy.loginfo('Recalculating path...')
        area_map, Yindexes, Xindexes, north, south, east, west = makePolygon(base_polygon) #Make bitmap from polygon
        start_points = get_random_coords(area_map, n) # Random start coordinates
        A, losses = darp(300, area_map, start_points, pbar=True) # Area division algorithm
        drone_maps = [get_drone_map(A,i) for i in range(n)] #assign a map for each drone
        coverage_paths = [bcd(drone_maps[i],start_points[i]) for i in range(n)]  #Calculate the routes for each drone
        old_polygon = base_polygon
#######
        '''
        imshow(A,1,4,1, figsize=(20,5))
        imshow_scatter(start_points,color="black")
        dist_maps = [dist_fill(drone_maps[i],[start_points[i]]) for i in range(n)]
        [imshow(dist_maps[i],1,4,i+2) for i in range(n)];

        for i in range(n):
            imshow(dist_maps[i],1,4,i+2)
            plot(coverage_paths[i],color="white",alpha=0.6)
            end_point = coverage_paths[i][-1]
            imshow_scatter(start_points[i], color="green")
            imshow_scatter(end_point, color="red")

#        plt.show()

        '''
        #################################################
        ### meter topic salida a aerostack drone111/#####
        #################################################
        coverage_path_gazebo = alg2gazebo(coverage_paths, Yindexes, Xindexes, north, south, east, west)
        # Drone with the maximum number of positions: Each drone has a different path with different number of points
        maxPos=0
        for drone in range(len(coverage_path_gazebo)):        # Number of drones
            numberPos = len(coverage_path_gazebo[drone])
            if numberPos>maxPos:
                maxPos=numberPos

        # Execute the function for every position and for all of the drones
        C2Path = Path();
        aerostackPath = Path();

        C2Path.header.frame_id = 'world'

        for pos in range(maxPos):                                # Maximum number of positions
            for drone in range(len(coverage_path_gazebo)):        # Number of drones
                # Try and except to deal with the problem of having different number of positions
                #try:
                    # Call the function publishTrajectory sending the number of the drone and the desired position
                    C2Path.poses.append(appendPath(coverage_path_gazebo[drone][pos], True))
                    aerostackPath.poses.append(appendPath(coverage_path_gazebo[drone][pos], False))
                    #rospy.loginfo('Drone '+ str(drone+1) + ' = ' + str(coverage_path_gazebo[drone][pos]))
                #except:
                    #rospy.loginfo('Drone ',drone+1, ' has reached its final position')

        pubC2.publish(C2Path)
        update = False
#        quit()
