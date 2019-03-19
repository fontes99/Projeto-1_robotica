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

v1 = 0.125
v2 = 0.5
w = pi/4

go = Twist(Vector3(v1,0,0), Vector3(0,0,0))
back = Twist(Vector3(-v2,0,0), Vector3(0,0,0))
go2 = Twist(Vector3(v2,0,0), Vector3(0,0,0))
stop = Twist(Vector3(0,0,0), Vector3(0,0,0))
turnR = Twist(Vector3(0,0,0), Vector3(0,0,-w))
turnL = Twist(Vector3(0,0,0), Vector3(0,0,w))
q=-1
# tg = 2.0
# tt = 1.0
tg = 1
tt = .5
timer = 0
def bateu(dado):
	global q
	q = dado.data
 	print(q)

if __name__=="__main__":
	rospy.init_node("evasiva.py")
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	hit = rospy.Subscriber("/bumper", UInt8, bateu)
	global q
	while not rospy.is_shutdown():
		print("Loop q: ", q)
		if q == 1:
			pub.publish(back)

			if q == 3:
				pub.publish(go2)
				rospy.sleep(tg)
				pub.publish(turnR)
				rospy.sleep(tt)
				q = 0

			if q == 4:
				pub.publish(go2)
				rospy.sleep(tg)
				pub.publish(turnL)
				rospy.sleep(tt)
				q = 0

			rospy.sleep(tg)
			pub.publish(turnR)
			rospy.sleep(tt)
			q = 0

		if q == 2:
			pub.publish(back)

			if q == 3:
				pub.publish(go2)
				rospy.sleep(tg)
				pub.publish(turnR)
				rospy.sleep(tt)
				q = 0

			if q == 4:
				pub.publish(go2)
				rospy.sleep(tg)
				pub.publish(turnL)
				rospy.sleep(tt)
				q = 0

			rospy.sleep(tg)
			pub.publish(turnL)
			rospy.sleep(tt)
			q = 0

		else:
		 	pub.publish(go)
		 	rospy.sleep(0.01)
# rospy.sleep(1.0)
