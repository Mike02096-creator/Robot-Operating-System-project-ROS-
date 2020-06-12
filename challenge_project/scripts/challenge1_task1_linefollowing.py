#!/usr/bin/env python
# -*- coding: utf-8 -*-
#include <ros/console.h>

##############################################################################################################
# Code du noeud utilisé dans le challenge 1 Task 1. Ce noeud est lancé dans challenge1_task1.launch et demande au robot de suivre la ligne jaune devant laquelle il apparaît puis de s'arrêter dans la cible verte. 

# Pour cela nous utilisons les informations fournies par la caméra du robot : definition d'un subscriber au topic de la camera fournissant un message de type Image  (sensor_msgs.msg). 
# Le informations sont ensuites traités, à l'aide de la librairie Open CV, dans la fonction 'callback' associée au subscriber précédement definit. 
# La definition d'un publisher sur le topic comportant les instructions de vitesse du robot permet ensuite de commander l'avancé du robot. 
##############################################################################################################

# Importation des librairies utiles :
import rospy 
import cv2 
import numpy as np 

# Importation des types de messages traités ici 
from sensor_msgs.msg import Image # Type de message pour les informations de la camera 
from geometry_msgs.msg import Twist # Type de message pour la commande de vitesse 

# Importation et création du materiel utile à la manipulation des données via la librairie cv2
from cv_bridge import CvBridge , CvBridgeError
bridge=CvBridge()


# Création d'un message de type Twist destiné à stocker les commandes de vitesses du robot 
twist=Twist()

########## Fonction callback ratachée au subscriber du topic de la camera
def callback(data):

	try:
		# Conversion de l'image récupérée en "open cv image"
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		# Gestion d'eventuelles erreures durant la conversion 
		print(e)
	
	# Dimensions de l'image récupérée 
	h, w, d = cv_image.shape

	# Conversion en mode hsv (hue saturation value)
	hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_RGB2HSV)

	# Nous voulions tracker la ligne jaune : il fallait donc trouver le code hsv correspondant au code RGB lu à l'aide de l'affichage camera. 
	#yellow=np.uint8([[[255,255,90 ]]])
	#hsv_yellow = cv2.cvtColor(yellow,cv2.COLOR_BGR2HSV)
	#print hsv_yellow 
	# resultat hsv : 90 165 255
	
	# Definition d'un espace hsv de couleurs assimilé au jaune pour le robot.  
	# (Les valeurs limites on été trouvés experimentalement par itérations) 
    	lower_yellow = np.array([80,50,50])
    	upper_yellow = np.array([100,255,255])

	# Création d'un masque hsv qui ne gardera que les parties de l'image assimilées au jaune.
	mask_white = cv2.inRange(hsv_image,lower_yellow,upper_yellow)

	# Calcul des moments du masque pour trouver le centre de la ligne (cx,cy)
	M =cv2.moments(mask_white)

	# Si du jaune est détécté à l'image le robot avance 
	if M['m00']>0:

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

	# Arret si pas de ligne détéctée 
 	else:
		twist.linear.x = 0
      		twist.angular.z = 0
		cmd_vel_pub.publish(twist)

	# Affichage caméra 
	cv2.namedWindow("window", 1)
	cv2.imshow("window",cv_image)
	cv2.waitKey(3)
########## Fonction callback : Fin
	

if __name__ == '__main__':

	# Récupération du paramètre topic de la commande vitesse, si pas de paramètre,
        # '/cmd_vel' = le nom le plus courant  
        if rospy.has_param('topic_velocity'):
            topic = rospy.get_param('topic_velocity')
        else :
            topic = '/cmd_vel'

	# Création d'un node challenge1_task1_linefollowing : 
	rospy.init_node('challenge1_task1_linefollowing', anonymous=True)

	# Récupération des paramètres de vitesse du robot 
	lin_v = rospy.get_param('/linear_scale')

	# Definition d'un subscriber au topic de la camera 
	rospy.Subscriber('/camera/image_raw', Image, callback)

	# Definition d'un publisher sur le topic de commande de vitesse du robot 
	cmd_vel_pub = rospy.Publisher(topic,Twist, queue_size=10)

	try:
        	# Attente d'arret du noeud ...
        	rospy.spin()

	except rospy.ROSInterruptException:
       		pass
