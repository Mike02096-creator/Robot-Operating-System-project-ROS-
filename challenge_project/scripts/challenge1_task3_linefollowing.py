#!/usr/bin/env python
# -*- coding: utf-8 -*-
#include <ros/console.h>

#---------------------------------------
# Note sur les commentaires : 
# Commentaire
#Ligne de code commentée
#---------------------------------------  

##############################################################################################################
# Code du noeud utilisé dans le challenge 1 Task 3. Ce noeud est lancé dans challenge1_task3.launch et demande au robot de suivre la ligne devant laquelle il apparaît en variant sa vitesse en fonction de la couleur de celle-ci (ralantissement significatif sur le rouge) mais aussi en fonction des difficultés du chemin (virages ou brouillage du circuit)  puis de s'arrêter dans la cible verte. 

# Pour cela nous suivons le même principe que dans le script challenge1_task2_linefollowing.py en afinant le condtions de commande de vitesses afin que le robot ne rique pas de se perdre. De plus un processus de recherche de ligne permet au robot de retrouver la ligne à suivre dans le cas ou il l'aurait perdu (ce phénomène est notamment observable lors d'un virage trés raide au niveau d'une piste brouillée).
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
	
	# Variable globale qui permet de savoir si le robot est en recherche de ligne ou non
	global en_recherche 
	# Variable globale qui sert pour la recherche de ligne : compte le nombre de tours de boucle réalisés pour empécher le robot de faire demi-tour : 
	global n_tours
	# Variable globale de la vitesse "maximum" (qui est ici modifiée pour lorsque le robot passe sur la piste brouillée)
	global lin_v
	# Variable globale qui indique si le robot est sur la piste brouillée :
	global mode_dark

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
	
	# De la même façon pour la ligne rouge : 
	#red =np.uint8([[[255,0,0 ]]])
	#hsv_red = cv2.cvtColor(red,cv2.COLOR_BGR2HSV)
	#print hsv_red 
	# resultat hsv : 120 255 255 
	lower_red = np.array([115, 250, 250])
	upper_red = np.array([125, 255, 255])

	# Pour le brouillage noir: 
	#noir =np.uint8([[[0,0,0 ]]])
	#hsv_noir = cv2.cvtColor(noir,cv2.COLOR_BGR2HSV)
	#print hsv_noir 
	# resultat hsv : 0 0 0
	lower_noir = np.array([0, 0, 0])
	upper_noir = np.array([0, 0, 0])

	# Et pour la cible verte  : 
	#green =np.uint8([[[0,255,0 ]]])
	#hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
	#print hsv_green 
	# resultat hsv : 60 255 255
	lower_green = np.array([55, 250, 250])
	upper_green = np.array([65, 255, 255]) 

	# Création d'un masque hsv qui ne gardera que les parties de l'image respectivement assimilées au jaune, au rouge, au noir et au vert.
	mask_yellow = cv2.inRange(hsv_image,lower_yellow,upper_yellow)
	mask_red = cv2.inRange(hsv_image,lower_red,upper_red)
	mask_noir = cv2.inRange(hsv_image,lower_noir,upper_noir)
	mask_green = cv2.inRange(hsv_image,lower_green,upper_green)
	
	# Calcul des moments des masques
	M_yellow =cv2.moments(mask_yellow)
	M_red = cv2.moments(mask_red)
	M_green = cv2.moments(mask_green)
	M_noir = cv2.moments(mask_noir)

	# Lorsque plus de jaune est détécté que de rouge le robot va aussi vite que les conditions lui permettent 
	if M_yellow['m00']>M_red['m00'] and M_yellow['m00']>0:
		# Calcul du "centre" de la ligne détéctée
		cx = int(M_yellow['m10']/M_yellow['m00'])
		cy = int(M_yellow['m01']/M_yellow['m00'])
		# Tracé d'un point sur l'image camera représentant ce centre
		cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)

		# Debut du contrôle 
		# (fonction  de l'écart horizontal du centre de la ligne par rapport au centre de l'image)
      		err = cx - w/2

			# Condition qui permet de sortir du mécanisme de recherche de ligne 
			# (quand une nouvelle ligne détéctée)
		if ((cy>=280) and (M_yellow['m00']>=300000) and (en_recherche==True)):
			print "Ligne retrouvée! Fin du processus."
			en_recherche = False
			twist.linear.x = 0
      			twist.angular.z = 0
			cmd_vel_pub.publish(twist)
			n_tours = 0

			# Mécanisme de recherche de ligne dans le cas ou le robot a perdu la ligne qu'il suivait 
			# (mais qu'il en detecte d'autres plus loin)
		if (cy < 280) or (M_yellow['m00']<300000) :
			print "Processus recherche de ligne activé ..."
			# Le robot tourne à droite, tant qu'il n'a pas réçut plus de 90 fois cet ordre
			# (valeur definie experimentalement) 
			if n_tours <90 : 
				twist.linear.x = 0
      				twist.angular.z = - 0.3
				cmd_vel_pub.publish(twist)
			# Si le robot n'a pas détécté de ligne à droite il cherche vers la gauche 
			else : 
				twist.linear.x = 0
      				twist.angular.z = 0.3
				cmd_vel_pub.publish(twist)
			n_tours +=1 
			en_recherche=True

			# Debut du contrôle adapté aux virages 
		elif abs(err)<=60 : # ligne trés droite 
      			twist.linear.x = lin_v
			twist.angular.z = -float(err)/100
			cmd_vel_pub.publish(twist)
		elif abs(err)>60 and abs(err)<=80 : # tourant leger (anticipation en cas d'un tournant plus fort)
			twist.linear.x = lin_v/2
			twist.angular.z = -float(err)/100
			cmd_vel_pub.publish(twist)
		elif abs(err)>80 and abs(err)<=105 : # tournant plus fort (anticipation en cas d'un tournant plus fort)
			twist.linear.x = lin_v/4
			twist.angular.z = -float(err)/100
			cmd_vel_pub.publish(twist)
		elif abs(err)>105 and abs(err)<=120 : # virage dur (anticipation en cas d'un tournant plus fort)
			twist.linear.x = lin_v/6
			twist.angular.z = -float(err)/100
			cmd_vel_pub.publish(twist)
		elif (M_yellow['m00']<20000000): # virage en épingle (cas d'urgence)
			twist.linear.x = lin_v/9
			twist.angular.z = -float(err)/100
			cmd_vel_pub.publish(twist)
		else : # virage trés dur
			twist.linear.x = lin_v/8
      			twist.angular.z = -float(err)/100
      			cmd_vel_pub.publish(twist)
      		# Fin du contrôle

	# Si il y a plus de rouge détécté que de jaune alors le robot avance plus lentement 
	elif M_yellow['m00']<M_red['m00'] and M_red['m00']>0:
		# Calcul du "centre" de la ligne détéctée
		cx = int(M_red['m10']/M_red['m00'])
		cy = int(M_red['m01']/M_red['m00'])
		# Tracé d'un point sur l'image camera représentant ce centre
		cv2.circle(cv_image, (cx, cy), 20, (255,255,255), -1)

		# Debut du contrôle
      		err = cx - w/2
      		twist.linear.x = lin_v/4
      		twist.angular.z = -float(err)/70
      		cmd_vel_pub.publish(twist)
      		# Fin du contrôle
	
	# Si la cible verte est détéctée assez proche alors le robot se stop  
	elif M_green['m00']>M_red['m00'] and M_green['m00']>M_yellow['m00']:
		# Arret sur la cible 
		twist.linear.x = 0
      		twist.angular.z = 0
		cmd_vel_pub.publish(twist)

	# Mécanisme de recherche de ligne dans le cas ou le robot ne détecte plus aucune ligne
	# (même principe que plus haut excépté que le robot commence par chercher à gauche) 
 	else:
		print "Processus recherche de ligne activé ..."
		if n_tours <90 : 
			twist.linear.x = 0
      			twist.angular.z = 0.3
			cmd_vel_pub.publish(twist)
		else : 
			twist.linear.x = 0
      			twist.angular.z = - 0.3
			cmd_vel_pub.publish(twist)
		n_tours +=1 
		en_recherche=True

	# Si une proportion importante de brouillage noir est détéctée la vitesse est réduite :
	# (cette condition n'est vérifiée qu'une fois grâce à la condition "mode_dark == False" ) 
	if  M_noir['m00']> 15000000 and mode_dark == False : 
		lin_v = lin_v/2
		mode_dark = True

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

	# Initialisation des variables globales :
	en_recherche = False 
	n_tours = 0
	mode_dark = False

	# Création d'un node challenge1_task3_linefollowing : 
	rospy.init_node('challenge1_task3_linefollowing', anonymous=True)

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
