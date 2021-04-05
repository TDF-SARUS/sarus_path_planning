#!/usr/bin/env python
# Input: coverage_path -> Vector of vectors with the coordinates of the path for all of the drones
# Output: publish a topic of ROS for each drone, every 5 seconds
# Function: publishTrajectory -> In charge of publishing coordiantes

import rospy
import tf
import std_msgs.msg
import time
#import numpy as np
from geometry_msgs.msg import PoseStamped

# Generate the coordinates vector of each drone (will be computed prior to this point)
coverage_path=[[(1,2),(3,4)],[(5,6),(7,8),(9,10)]]

# Frequency of the sleep
rate = rospy.Rate(0.2) #0.2 Hz -> 5s

# Drone with the maximum number of positions: Each drone has a different path with different number of points
maxPos=0
for drone in range(len(coverage_path)):		# Number of drones
    numberPos = len(coverage_path[drone]))
    if numberPos>maxPos:
        maxPos=numberPos

# Execute the function for every position and for all of the drones
for pos in maxPos:	                            # Maximum number of positions
    for drone in range(len(coverage_path)):		# Number of drones
        # Try and except to deal with the problem of having different number of positions
        try:
            # Call the function publishTrajectory sending the number of the drone and the desired position
            publishTrajectory(drone+1, coverage_path[drone][pos])
            print('Drone ',drone+1, ' = ', coverage_path[drone][pos])
            rate.sleep(5) # 1s
        except:
            print('Drone ',drone+1, ' has reached its final position')
    print()
    rate.sleep() # 5s

# ------------------ FUNCTION publish Trajectory (created by Oscar and Jorge) -----------------------
	# Inputs: droneNum=Number of drone   |   coordinates=Desired position of the drone
	# Output: publish a topic of ROS for the particular droneNum drone

def publishTrajectory(droneNum, coordinates):
	# Define the name of the drone's topic
	topic='/drone11'+str(droneNum)+'/motion_reference/pose'
	# Define the publisher and type of message
	pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
	rospy.init_node('test', anonymous=True)
	
	msg = PoseStamped()

    # Also possible with numpy
    #[x_ref, y_ref] = np.split(np.array(coordinates), 2, axis = 1)
	
	# Separate the position in its coordinates
	msg.pose.position.x=coordinates[0]
	msg.pose.position.y=coordinates[1]
	msg.pose.position.z=1.0 #coordinates[2]

	# Publish the message (coordinates of the position of drone droneNum)
	pub.publish(msg)


	if __name__ == '__main__':
	try:
		publishTrajectory()
	except rospy.ROSInterruptException:
		pass
