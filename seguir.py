#! /usr/bin/env python
# -*- coding:utf-8 -*-
'''
roscore
roslaunch turtlebot3_bringup turtlebot3_remote.launch
rosrun topic_tools relay /raspicam_node/image/compressed /kamera

rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rqt_image_view

'''


__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cormodule
import cormodule2
from std_msgs.msg import UInt8
from math import pi

### ---------- FUNÇÕES -----------------

def roda_todo_frame(imagem):
	print("frame")
	global area
	global cv_image
	global media
	global centro

	global area2
	global cv_image2
	global media2
	global centro2

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	# print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		cv_image2 = cv_image
		media, centro, area =  cormodule.identifica_cor(cv_image)
		media2, centro2, area2 =  cormodule2.identifica_cor(cv_image2)
		# print(media)
		cv_image = cv2.add(cv_image,cv_image2)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	
def bateu(dado):
	global q
	q = dado.data
 	print(q)

def bumper(porta):

	if porta == 1:
		
		pub.publish(back)

		if porta == 3:
			pub.publish(go2)
			rospy.sleep(tg)
			pub.publish(turnR)
			rospy.sleep(tt)
			
		if porta == 4:
			pub.publish(go2)
			rospy.sleep(tg)
			pub.publish(turnL)
			rospy.sleep(tt)
			
		rospy.sleep(tg)
		pub.publish(turnR)
		rospy.sleep(tt)
		
	elif porta == 2:
		pub.publish(back)

		if porta == 3:
			pub.publish(go2)
			rospy.sleep(tg)
			pub.publish(turnR)
			rospy.sleep(tt)
			

		if porta == 4:
			pub.publish(go2)
			rospy.sleep(tg)
			pub.publish(turnL)
			rospy.sleep(tt)
			

		rospy.sleep(tg)
		pub.publish(turnL)
		rospy.sleep(tt)

	return 0

### ----------------- DECLARAÇÃO DE VARIÁVEIS -----------------------

v1 = 0.125
v2 = 0.5
w = pi/4

go = Twist(Vector3(v1,0,0), Vector3(0,0,0))
back = Twist(Vector3(-v2,0,0), Vector3(0,0,0))
go2 = Twist(Vector3(v2,0,0), Vector3(0,0,0))
stop = Twist(Vector3(0,0,0), Vector3(0,0,0))
turnR = Twist(Vector3(0,0,0), Vector3(0,0,-w))
turnL = Twist(Vector3(0,0,0), Vector3(0,0,w))

tg = 1
tt = .5
timer = 0
q = -1


bridge = CvBridge()

TelaMeio = 320
margin = 20

cv_image = None
media = []
centro = []
atraso = 0.1E10 # 1 segundo e meio. Em nanossegundos
area = 0.0 # Variavel com a area do maior contorno

cv_image = None
media2 = []
centro2 = []
area2= 0.0 # Variavel com a area do maior contorno



# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

# A função a seguir é chamada sempre que chega um novo frame

### -------------------------- PROGRAMA EM SI -------------------------------

if __name__=="__main__":
	rospy.init_node("seguir")

	topico_imagem = "/kamera"
	
	# Para renomear a *webcam*
	# 
	# 	rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera
	# 
	# Para renomear a câmera simulada do Gazebo
	# 
	# 	rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera
	# 
	# Para renomear a câmera da Raspberry
	# 
	# 	rosrun topic_tools relay /raspicam_node/image/compressed /kamera
	# 

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	# print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	hit = rospy.Subscriber("/bumper", UInt8, bateu)

	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

			print(q)

			if q != 0:

				q = bumper(q)

			if area > area2:
				if len(media) != 0 and len(centro) != 0:

					if media[0] < TelaMeio + margin:
						vel = Twist(Vector3(0.075,0,0),Vector3(0,0,-0.1))

					elif media[0] > TelaMeio - margin:

						vel = Twist(Vector3(0.075,0,0),Vector3(0,0,0.1))

					else:
						vel = Twist(Vector3(0.1,0,0),Vector3(0,0,0))
			elif area2 > area:
				if len(media) != 0 and len(centro) != 0:

					if media2[0] < TelaMeio + margin:
						vel = Twist(Vector3(-0.075,0,0),Vector3(0,0,-0.1))

					elif media2[0] > TelaMeio - margin:

						vel = Twist(Vector3(-0.075,0,0),Vector3(0,0,0.1))

					else:
						vel = Twist(Vector3(-0.1,0,0),Vector3(0,0,0))
			else:

				vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))


			velocidade_saida.publish(vel)

			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
	    vel = Twist(Vector3(0,0,0),Vector3(0,0,0))
	    velocidade_saida.publish(vel)


