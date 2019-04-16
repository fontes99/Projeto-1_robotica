#! /usr/bin/env python
# -*- coding:utf-8 -*-
'''
roscore
roslaunch turtlebot3_bringup turtlebot3_remote.launch
rosrun topic_tools relay /raspicam_node/image/compressed /kamera

rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rqt_image_view


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
'''
__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
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
from sensor_msgs.msg import LaserScan
from math import pi
import visao_module

### ---------- FUNÇÕES -----------------
def identifica_imagens(imagem):
	print("frame")
	global objeto
	global cv_image
	global controImg
	global viu_Obj
	global ObjPos

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		controImg, imagem, resultados =  visao_module.processa(cv_image)
		for r in resultados:
			if r[0] == objeto:
				print("Ini",r[2],"Fin",r[3],r[0])
				ObjPos = r[2][0] + ((r[3][0] - r[2][0])/2)
				viu_Obj == True
		depois = time.clock()
	except CvBridgeError as e:
		print('ex', e)

def identifica_cor(imagem):
	global areaCor
	global cv_image
	global mediaCor
	global centroCor
	global viu_cor

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		mediaCor, centroCor, areaCor =  cormodule.identifica_cor(cv_image,margin)
		print(mediaCor)
		viu_cor = True
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	
def bateu(dado):
	global q
	q = dado.data

def escaneia(dado):
	global ang
	index = 0
	print("Scaner")
	dist_minina = 0.3
	print("Faixa valida: ", dado.range_min , " - ", dado.range_max)
	for i in dado.ranges:
		if i <= dist_minina and i > dado.range_min:
			print(i)
			ang = index
			break
		else:
			index += 1

def bumper(porta):
	print("Bumper")
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
bridge = CvBridge()
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
	"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
	"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
	"sofa", "train", "tvmonitor"]
for i in range(len(CLASSES)):
	print(i, CLASSES[i])
objeto = int(input("Indice de escolha: \n"))
objeto = CLASSES[objeto]
ObjPos = 0
ang = 0

obstaculo = False

viu_Obj = False
check_delay = False
viu_cor = False

w = pi/6

v1 = 0.125
v2 = 0.5
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

v = 0.1
TelaMeio = 320
margin = 20

mediaCor = [0]
centroCor = [0]
atraso = 0.1E9
areaCor = 0.0

cv_image = None
vel = Twist(Vector3(v/4,0,0), Vector3(0,0,0))
### -------------------------- PROGRAMA EM SI -------------------------------

if __name__=="__main__":
	rospy.init_node("seguir")
	topico_imagem = "/kamera"
	recebedor_Cor = rospy.Subscriber(topico_imagem, CompressedImage, identifica_cor, queue_size=4, buff_size = 2**24)
	recebedor_Img = rospy.Subscriber(topico_imagem, CompressedImage, identifica_imagens, queue_size=4, buff_size = 2**24)

	pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	hit = rospy.Subscriber("/bumper", UInt8, bateu)
	laser = rospy.Subscriber("/scan",LaserScan, escaneia)

	try:
		while not rospy.is_shutdown():
			if q != 0:
				print('5')
				q = bumper(q)

			elif ang != 0:
				print('6')
				if ang < 90:
					print('7')
					vel = Twist(Vector3(-v,0,0),Vector3(0,0,w))
				elif ang < 180:
					print('8')
					vel = Twist(Vector3(v,0,0),Vector3(0,0,-w))
				elif ang < 270:
					print('9')
					vel = Twist(Vector3(v,0,0),Vector3(0,0,w))
				elif ang >= 270:
					print('10')
					vel = Twist(Vector3(-v,0,0),Vector3(0,0,-w))
				else:
					print('11')
					vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
				ang = 0
			elif ObjPos > 0:
				if ObjPos < TelaMeio - margin:
					print('12')
					vel = Twist(Vector3(v,0,0),Vector3(0,0,-w))
				elif ObjPos > TelaMeio - margin and ObjPos < TelaMeio + margin:
					print('13')
					vel = Twist(Vector3(v,0,0),Vector3(0,0,0))
				elif ObjPos > TelaMeio + margin:
					print('14')
					vel = Twist(Vector3(v,0,0),Vector3(0,0,w))
				ObjPos = 0
			elif areaCor > 5000:
				if mediaCor[0] < TelaMeio - margin:
					print('1')
					vel = Twist(Vector3(-v,0,0),Vector3(0,0,w))
				elif mediaCor[0] >= TelaMeio - margin and mediaCor[0] <= TelaMeio + margin:
					print('2')
					vel = Twist(Vector3(-v,0,0),Vector3(0,0,0))
				elif mediaCor[0] > TelaMeio + margin:
					print('3')
					vel = Twist(Vector3(-v,0,0),Vector3(0,0,-w))

			else:
				print('15')
				print("Entering Sentry mode")
				vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
			print("Publish")
			pub.publish(vel)
			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
	    vel = Twist(Vector3(0,0,0),Vector3(0,0,0))
	    pub.publish(vel)