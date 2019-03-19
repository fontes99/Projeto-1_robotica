#! /usr/bin/env python
# -*- coding:utf-8 -*-

'''

roscore

roslaunch turtlebot3_bringup turtlebot3_remote.launch

'''

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import UInt8
from math import pi


v = 0.125
w = pi/4


go = Twist(Vector3(v,0,0), Vector3(0,0,0))
back = Twist(Vector3(-v,0,0), Vector3(0,0,0))
stop = Twist(Vector3(0,0,0), Vector3(0,0,0))
turnR = Twist(Vector3(0,0,0), Vector3(0,0,-w))
turnL = Twist(Vector3(0,0,0), Vector3(0,0,w))


ts = 1.0
tg = 2.0
tt = 4.0


q = 0

def bateu(dado):

	global q

	q = None
	q = dado.data


if __name__=="__main__":

	rospy.init_node("le_scan")

	pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	hit = rospy.Subscriber("/bumper", UInt8, bateu)



	while not rospy.is_shutdown():

		print(q)
		
		if q == 1:

			q = None

			pub.publish(stop)
			rospy.sleep(ts)

			pub.publish(back)
			rospy.sleep(tg)

			pub.publish(turnR)
			rospy.sleep(tt)

		if q == 2:

			q = None
				
			pub.publish(stop)
			rospy.sleep(ts)

			pub.publish(back)
			rospy.sleep(tg)

			pub.publish(turnL)
			rospy.sleep(tt)
		
		if q == 3:

			q = None

			pub.publish(stop)
			rospy.sleep(ts)

			pub.publish(go)
			rospy.sleep(tg)

			pub.publish(turnL)
			rospy.sleep(tt)
		
		if q == 4:

			q = None

			pub.publish(stop)
			rospy.sleep(ts)

			pub.publish(go)
			rospy.sleep(tg)

			pub.publish(turnR)
			rospy.sleep(tt)


		else:

		 	pub.publish(go)
		 	# rospy.sleep(1.0)