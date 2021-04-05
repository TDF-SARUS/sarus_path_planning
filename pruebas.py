#!/usr/bin/env python
#El objetivo de este codigo es mandar coordenadas para un control de posicion del proyecto teleoperation_gazebo (seleccionar Pose Mode controller)
#Elaborado por Oscar Ramos, cualquier duda preguntar a el o a cualquiera del grupo de Mission Planning de TDF

import rospy
import tf
import std_msgs.msg
import numpy as np
from geometry_msgs.msg import PoseStamped

#Aquí simulamos que nos están mandando una trayectoria de algún modo
def ReceiveTrajectory():
	coords = []
	for t in range(0,100):
		if (t<5):
			coords.append([1.0, 1.0])
		if (t>=5 and t<10):
			coords.append([-1.0, 0.0])
		if (t>=10):
			coords.append([0.0, 0.0])

	return coords

def publishTrajectory():
	t=0
	pub = rospy.Publisher('/drone111/motion_reference/pose', PoseStamped, queue_size=10)
	rospy.init_node('test', anonymous=True)
	rate = rospy.Rate(1) #1 Hz
	msg = PoseStamped()
	while not rospy.is_shutdown():

		# Recibimos la trayectoria y la dividimos en coordenadas x e y
		data = ReceiveTrajectory()
		[x_ref, y_ref] = np.split(np.array(data), 2, axis = 1)

		# Enviamos el último valor y lo borramos
		msg.pose.position.x=x_ref.pop(-1)
		msg.pose.position.y=y_ref.pop(-1)

		rospy.sleep(1)
		pub.publish(msg)
		rate.sleep()
		t=t+1


if __name__ == '__main__':
	try:
		publishTrajectory()
	except rospy.ROSInterruptException:
		pass