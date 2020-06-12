#!/usr/bin/env python
# -*- coding: utf-8 -*-
#include <ros/console.h>

##############################################################################################################
# Code du noeud utilisé dans le challenge 3 Task 1. Ce noeud est lancé dans challenge3_task1.launch et demande au robot de 
# suivre une ligne jaune. Le robot doit également s'arréter en face du premier mur à une distance suffisante en attendant son ouverture
# et ensuite s'arréter de nouveau devant le mur bleu (porte de garage) pour lui envoyer un signal d'ouverture.

# Le principe de base de ce code (publisher, subscriber ...) reste semblable aux challenge 1 et 2. 
# Cepdendant, on ajoute un publisher 'Garage_Opener_Door' pour que le robot emette le signal d'ouverture et on utilise le scan pour que le robot
# s'arrête devant les obstacles.
##############################################################################################################


# Importation des librairies utiles :
import rospy 
import cv2 # import open CV library
import numpy as np # import numpy library
from cv_bridge import CvBridge , CvBridgeError

# Importation des types de messages traités ici 
from geometry_msgs.msg import Twist # Type de message pour le topic de commande de vitesse 
from sensor_msgs.msg import LaserScan # Type de message pour le topic du scan 
from sensor_msgs.msg import Image # Type de message pour le topic de la caméra 
from std_msgs.msg import Bool # Type de message pour le topic d'ouverture de la porte 

# Création d'un message de type Twist destiné à stocker les commandes de vitesses du robot 
twist=Twist()
# Création d'un message de type Bool destiné à stocker les commandes d'émission du signal pour ouvrir une porte du robot 
door_key = Bool()
# Création d'un message de type CvBridge destiné à covertire les images ROS vues par la caméra du robot en image de type OpenCV 
bridge=CvBridge()


########## Fonction callback ratachée au subscriber du topic de la camera
def callback_cam(data):

	# Variable globale qui indique si la voie est libre (mur levé et free_way == True) ou si un mur bloque le chemin du robot (free_way == False)
	global free_way

	try:
		# Conversion de l'image récupérée en "open cv image"
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)
	
	# Dimensions de l'image récupérée 
	h, w, d = cv_image.shape

	# Conversion en mode hsv (hue saturation value)
	hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_RGB2HSV)

	# Definition d'un espace hsv de couleurs assimilé au jaune pour le robot.  
	# (Les valeurs limites on été trouvés experimentalement par itérations) 
    	lower_yellow = np.array([80,50,50])
    	upper_yellow = np.array([100,255,255])

	# De la même façon pour le mur bleu : 
	#blue =np.uint8([[[0,0,255 ]]])
	#hsv_blue = cv2.cvtColor(blue,cv2.COLOR_BGR2HSV)
	#print hsv_blue 
	# Resultat hsv : [0 255 255] 
	lower_blue = np.array([0, 100, 100])
    	upper_blue = np.array([20, 255, 255])

	# Création d'un masque hsv qui ne gardera que les parties de l'image respectivement assimilées au jaune et au bleu.
	mask_yellow = cv2.inRange(hsv_image,lower_yellow,upper_yellow)
	mask_blue = cv2.inRange(hsv_image,lower_blue,upper_blue)

	# Calcul des moments des masques
	M_yellow =cv2.moments(mask_yellow)
	M_blue = cv2.moments(mask_blue)

    	# Si du jaune est détécté à l'image et que la voie est libre le robot avance
	if M_yellow['m00']>0 and free_way==True:
        	# Calcul du "centre" de la ligne détéctée
		cx = int(M_yellow['m10']/M_yellow['m00'])
		cy = int(M_yellow['m01']/M_yellow['m00'])
        	# Tracé d'un point sur l'image camera représentant ce centre
		cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)

		# Debut du contrôle (proportionnel à l'écart horizontal du centre de la ligne par rapport au centre de l'image)
      		err = cx - w/2
      		twist.linear.x = lin_v
      		twist.angular.z = -float(err)/100 
      		cmd_vel_pub.publish(twist)
      		# Fin du contrôle
    
    	# Si du bleu est détécté à l'image c'est que la porte de garage est détecté. Le robot s'arrête donc et envoie le signal d'ouverture.      
	elif M_blue['m00']>0 : 
		print 'Ouvre la porte !'
		cmd_opendoor.publish(True)
		# Arret en attente d'ouverture 
		twist.linear.x = 0
      		twist.angular.z = 0
		cmd_vel_pub.publish(twist)
        
    # Arret si pas de ligne détéctée 
 	else:
		# Arret  
		twist.linear.x = 0
      		twist.angular.z = 0
		cmd_vel_pub.publish(twist)
    
    # Affichage caméra
	cv2.namedWindow("window", 1)
	cv2.imshow("window",cv_image)
	cv2.waitKey(3)
########## Fonction callback_cam : Fin

########## Fonction callback rattachée au subscriber du topic du scan (LIDAR)
def callback_scan(data):

	# Paramètres globaux 
	global safe_dist # Distance de sécurité
	global free_way 
	
	# Distance du robot à l'obstacle : 
	d1 = data.ranges[0:5]
	d2 = data.ranges[355:360]
	d = (sum(d1) + sum(d2))/10

    # Si la distance du robot au mur est supérieur à la safe_dist, c'est que il n'y a pas de mur devant et que la voie est libre.
	if d> safe_dist : 
		free_way = True
	else : 
		free_way = False 
	
if __name__ == '__main__':

	# Récupération du paramètre commande vitesse, si pas de paramètre,
	#'/cmd_vel' = le nom le plus courant  
	if rospy.has_param('topic_velocity'):
        	topic = rospy.get_param('topic_velocity')
	else :
        	topic = '/cmd_vel'

	# Récupération du paramètre de distance de sécurité 
	# safe_dist = 1 par defaut 
	if rospy.has_param('safety_dist'):
        	safe_dist = rospy.get_param('safety_dist')
	else :
		safe_dist = 1

	# Paramétre qui indique si la voie est libre (pas d'obstacle sur la route) 
	free_way = True  

	# Récupération des paramètres de vitesse du robot 
	lin_v = rospy.get_param('/linear_scale')

	# Creation d'un node : 
	rospy.init_node('challenge3_task1_Circuit', anonymous=True)
	
	# Definition d'un publisher sur le topic de commande de vitesse du robot 
	cmd_vel_pub = rospy.Publisher(topic,Twist, queue_size=10)

	# Definition d'un publisher pour l'ouverture de porte 
	cmd_opendoor = rospy.Publisher('/Garage_Door_Opener',Bool, queue_size=10)
	
	# Récupération du paramètre topic du scan, si pas de paramètre,
	#'/scan' = le nom le plus courant  
	if rospy.has_param('topic_scan'):
		topic_scan = rospy.get_param('topic_scan')
	else :
		topic_scan = '/scan'

	# Definition d'un subscriber au topic du LIDAR 
	rospy.Subscriber(topic_scan, LaserScan, callback_scan)

	# Récupération du paramètre topic de la caméra, si pas de paramètre,
	#'/camera/image_raw' = par defaut   
	if rospy.has_param('topic_cam'):
		topic_cam = rospy.get_param('topic_cam')
	else :
		topic_cam = '/camera/image_raw'

	# Definition d'un subscriber au topic de la camera 
	rospy.Subscriber(topic_cam, Image, callback_cam)


	try:
        	# Attente d'arret du noeud ...
        	rospy.spin()

	except rospy.ROSInterruptException:
       		pass
