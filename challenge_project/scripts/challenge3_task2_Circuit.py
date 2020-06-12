#!/usr/bin/env python
# -*- coding: utf-8 -*-
#include <ros/console.h>

##############################################################################################################
# Code du noeud utilisé dans le challenge 3 Task 2. Ce noeud est lancé dans challenge3_task2.launch et demande au robot de réaliser le circuit proposé comprenant : suivis de ligne, attente et demande d'ouverture de porte et passage dans un corridor.

# Ce code reprend tout les principes vu dans les challenges et tâches précédentes. Les principes permettant l'ouverture des portes sont détaillés dans la tache précédente de ce challenge. A cela à été ajouté un mécanisme permettant de passer a travers le corridor en utilisant le laser puis de repasser en mode camera une fois le corridor fini. 
    
# Une large utilisation des variables globales est faite pour indiquer au robot dans quelle situation particulière il se trouve mais aussi faciliter la communication des informations entre les callabcks du scanner et de la camera.   
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

################ Callback Cam #####################################################################
def callback_cam(data):

	# Variable globale qui indique si la voie est libre (mur levé et free_way == True) 
	# ou si un mur bloque le chemin du robot (free_way == False) 
	global free_way 
	# Variable globale qui permet d'activer la vue laser lorsque le robot ne detecte plus d'image exploitable par la camera
	# (Si les conditions pour le vue laser sont réunies)
	global laser_view
	# Variable globale qui indique si le robot se trouve actuellement sur un ligne rouge ou jaune
	# Elle est utile pour declancher un processus de recherche de ligne particulier à la sortie delicate du corridor
	global sur_ligne
	# Indique si le robot est 'perdu' (c'est à dire qu'il ne detecte ni ligne ni mur ) 
	# et permet d'eclancher un processus de recherche de ligne semblable à celui utilisé dans le challenge 1 tache 3.
	global lost
	# Variable globale qui permet de savoir si le robot est actuellement en recherche de ligne ou non
	global en_recherche 
	# Variable qui sert pour la recherche de ligne classique (or recherche d'obstacle orange) : 
	# compte le nombre de tours de boucle réalisées pour empécher le robot de faire un tours sur lui même 
	global n_tours
	# Indique que le robot a détécté une intersection en face de lui 
	# Permet de arretter le robot à la fin de la tache 2
	global nego_intersection

	# Vitesse de référence initialisée à la vitesse "maximum" 
	lin_v = lin_v_max 

	####### Initialisation du traitement des images camera
	try:
		# Conversion from image to open cv image
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)
	
	# Dimensions de l'image récupérée 
	h, w, d = cv_image.shape

	# Conversion en mode hsv (hue saturation value)
	hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_RGB2HSV)

	# Definition d'un espace de jaune en couleur HSV
    	lower_yellow = np.array([80,50,50])
    	upper_yellow = np.array([100,255,255])

	# Definition d'un espace de bleu en couleur HSV
	lower_blue = np.array([0, 100, 100])
    	upper_blue = np.array([20, 255, 255])

	# definition d'un espace de rouge en couleur HSV 
	lower_red = np.array([120, 255, 255])
	upper_red = np.array([120, 255, 255])

	# Creation des masques pour ne conserver que les couleur considérées 
	mask_yellow = cv2.inRange(hsv_image,lower_yellow,upper_yellow)
	mask_blue = cv2.inRange(hsv_image,lower_blue,upper_blue)
	mask_red = cv2.inRange(hsv_image,lower_red,upper_red)
 
	# Selection de zones décisives pour la negociation (arret) d'intersection (masque jaune)
	mask_yellow_inter_d = mask_yellow[4*h/6:5*h/6, 9*w/12:10*w/12]
		# Représenté en vert sur l'image caméra
	cv2.rectangle(cv_image, (9*w/12,4*h/6), (10*w/12,5*h/6), (0,255,0), thickness=1, lineType=8, shift=0)

	mask_yellow_inter_g = mask_yellow[4*h/6:5*h/6, 2*w/12:3*w/12]
		# Représenté en vert sur l'image caméra
	cv2.rectangle(cv_image, (2*w/12,4*h/6), (3*w/12,5*h/6), (0,255,0), thickness=1, lineType=8, shift=0)
	

	# Calcul des moments des différents masques 
	M_yellow =cv2.moments(mask_yellow)
	M_blue = cv2.moments(mask_blue)
	M_red = cv2.moments(mask_red)

	M_yellow_inter_d = cv2.moments(mask_yellow_inter_d)
	M_yellow_inter_g = cv2.moments(mask_yellow_inter_g)


	# Dans le cas où une intersection est détéctée (possibilitée d'aller à gauche ou à droite) arret du robot
	if (M_yellow_inter_d['m00']>100000 and M_yellow_inter_g['m00']>100000)  :
		print 'Intersection détéctée' 
		twist.linear.x = 0
    		twist.angular.z = 0
      		cmd_vel_pub.publish(twist)
		nego_intersection = True
	
	# Si le robot n'est pas sur la ligne (sortie du corridor) et qu'il détécte sufisament de la ligne à rejoindre
	elif sur_ligne == False and (M_yellow['m00']>700000 or M_red['m00']>900000) and nego_intersection==False : 
		print 'Visual Navigation'
		print 'Retour sur la ligne à suivre'
		cx = int(M_yellow['m10']/M_yellow['m00'])
		cy = int(M_yellow['m01']/M_yellow['m00'])
		cv2.circle(cv_image, (cx, cy), 20, (255,255,255), -1)

		# Debut du contrôle
      		err = cx - w/2
      		twist.linear.x = lin_v
    		twist.angular.z = -float(err)/70
      		cmd_vel_pub.publish(twist)
      		# Fin du contrôle
		
		# Si assez de jaune est détécté c'est que le robot est revenu sur la ligne
		if M_yellow['m00']>5000000:
			print 'De nouveau sur la ligne !'
			sur_ligne = True

	# Dans le cas où le robot est déja sur une ligne à suivre et qu'il détécte une couleur à la caméra 
	elif sur_ligne==True and (M_yellow['m00']>0 or M_blue['m00']>0 or M_red['m00']>0) and nego_intersection==False : 
		# Visual based navigation 
		print 'Visual Navigation'
		laser_view = False # (Laser navigation désactivée)

		# Dans le cas ou le robot detecte une ligne jaune à suivre et qu'il n'a pas d'obstacle en face de lui 
		# (Ou qu'il se trouve en recherche de ligne ou bien entrain de négocier un intersection) 
		if ((M_yellow['m00']>M_red['m00'] or en_recherche == True) and free_way==True):
			# Calcul des coordonnées du point à rejoindre
			cx = int(M_yellow['m10']/M_yellow['m00'])
			cy = int(M_yellow['m01']/M_yellow['m00']) 
			cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)

			### Debut du contrôle ############################################
      			err = cx - w/2

      				# Condition qui permet de sortir du mécanisme de recherche de ligne 
				# (quand une nouvelle ligne détéctée)
			if ((cy>=280) and (M_yellow['m00']>=300000) and (en_recherche==True)):
				print "Ligne retrouvée! Fin du processus."
				en_recherche = False
				twist.linear.x = 0
      				twist.angular.z = 0
				cmd_vel_pub.publish(twist)
				# Mise à zero des variables globales 
				n_tours = 0
				
			# Mécanisme de recherche de ligne dans le cas ou le robot a perdu la ligne qu'il suivait 
			# (bien en detecte d'autres plus loin)
			if (cy < 280) or (M_yellow['m00']<300000) :
				print "Processus recherche de ligne activé ..."

				# Le robot tourne dans un sens (droite)
				# tant qu'il n'a pas réçut plus de 90 fois cet ordre
				# (valeur definie experimentalement)
				if n_tours <90 : 
					twist.linear.x = 0
      					twist.angular.z = -0.3
					cmd_vel_pub.publish(twist)

				# Si le robot n'a pas détécté de ligne dans un sens il cherche dans l'autre 
				else : 
					twist.linear.x = 0
      					twist.angular.z = 0.3
					cmd_vel_pub.publish(twist)

				n_tours +=1 
				en_recherche=True

			# Negociation des virages classiques 
			elif abs(err)<=60 : # ligne trés droite
				print 'suivis de ligne'
      				twist.linear.x = lin_v
				twist.angular.z = -float(err)/100
				cmd_vel_pub.publish(twist)
			elif abs(err)>60 and abs(err)<=80 : # tourant leger (anticipation en cas d'un tournant plus fort)
				print 'suivis de ligne'
				twist.linear.x = lin_v/2
				twist.angular.z = -float(err)/100
				cmd_vel_pub.publish(twist)
			elif abs(err)>80 and abs(err)<=105 : # tournant plus fort (anticipation en cas d'un tournant plus fort)
				print 'suivis de ligne'
				twist.linear.x = lin_v/4
				twist.angular.z = -float(err)/100
				cmd_vel_pub.publish(twist)
			elif abs(err)>105 and abs(err)<=120 : # virage dur (anticipation en cas d'un tournant plus fort)
				print 'suivis de ligne'
				twist.linear.x = lin_v/6
				twist.angular.z = -float(err)/100
				cmd_vel_pub.publish(twist)
			elif (M_yellow['m00']<20000000): # virage en épingle (cas d'urgence)
				print 'suivis de ligne'
				twist.linear.x = lin_v/9
				twist.angular.z = -float(err)/100
				cmd_vel_pub.publish(twist)
			else : # virage trés dur
				print 'suivis de ligne'
				twist.linear.x = lin_v/8
      				twist.angular.z = -float(err)/100
      				cmd_vel_pub.publish(twist)
      			### Fin du contrôle ##############################################

		# Si le robot detecte une ligne rouge à suivre il avance moins vite  
		elif ((M_yellow['m00']<M_red['m00']) and M_red['m00']>0) and nego_intersection==False :
			
			cx = int(M_red['m10']/M_red['m00'])
			cy = int(M_red['m01']/M_red['m00'])
			cv2.circle(cv_image, (cx, cy), 20, (255,255,255), -1)

			# Debut du contrôle
      			err = cx - w/2
      			twist.linear.x = lin_v/4
			twist.angular.z = -float(err)/70
      			cmd_vel_pub.publish(twist)
			# Fin du contrôle
      			
		# Si le robot detecte la porte bleue il s'arrète et lui demande de s'ouvrir
		elif M_blue['m00']>0 and nego_intersection==False : 
			print 'Ouvre la porte !'
			cmd_opendoor.publish(True)
			# Arret et demande d'ouverture 
			twist.linear.x = 0
      			twist.angular.z = 0
			cmd_vel_pub.publish(twist)

		# Sinon le robot attent l'ouverture de la porte 
		else: 
			print 'Attente ouverture porte'
			# Arret en attente d'ouverture 
			twist.linear.x = 0
      			twist.angular.z = 0
			cmd_vel_pub.publish(twist)
		

	# Mécanisme de recherche de ligne dans le cas ou le robot ne détecte plus aucune ligne (même principe que précédemment) 
	elif lost == True and nego_intersection==False:
		print "Processus recherche de ligne activé ..."
		if n_tours <90 : 
			twist.linear.x = 0
      			twist.angular.z = -0.3
			cmd_vel_pub.publish(twist)
		else : 
			twist.linear.x = 0
      			twist.angular.z = 0.3
			cmd_vel_pub.publish(twist)
		n_tours +=1 
		en_recherche=True

	# Si aucune des conditions précédentes n'a été remplie le robot va verfier si il doit passer en laser_view
	elif nego_intersection==False:
		laser_view = True
	# Arret en fin de tache 
	else : 
		twist.linear.x = 0
      		twist.angular.z = 0
		cmd_vel_pub.publish(twist)


	# Fenêtre caméra 
	cv2.namedWindow("window", 1)
	cv2.imshow("window",cv_image)
	cv2.waitKey(3)
################ Callback Cam Fin ##################################################################

################ Callback Scan #####################################################################
def callback_scan(data):

	# Paramètres globaux 
	global safe_dist 
	global free_way 
	global laser_view
	global lost 
	global sur_ligne

	#Distance du robot à l'obstacle : 
	d1 = data.ranges[0:5]
	d2 = data.ranges[355:360]
	d = (sum(d1) + sum(d2))/10

	# Vitesse de référence initialisée à la vitesse "maximum" 
	lin_v = lin_v_max 

	# Verification pour que le robot ne passe pas en laser view alors qu'il a juste perdu la ligne :
	test1 = min(data.ranges[0:70])
	test2 = min(data.ranges[290:360])
	test = min(test1,test2)
		# Si le laser_view a été activé mais que aucun obstacle n'est détécté à moins de 4 m : 
		# le robot est perdu et doit recherche une ligne 
	if test >=4 and laser_view==True : 
		commande_laser = False
		lost = True
		# Si le laser_view a été activé et qu'un obstacle est détécté à moins de 4m le robot passe en laser_view
	elif laser_view == True and test <4  :
		commande_laser = True
		# Sinon le robot ne doit pas passé en laser_view
	else : 	
		commande_laser = False

	# test_b permet de verifier que le robot ne se trouve pas dangereusement prés d'un mur 
	test1_b = min(data.ranges[0:70])
	test2_b = min(data.ranges[290:360])
	test_b = min(test1_b,test2_b)


	# Distance du robot par rapport aux murs à gauche et à droite 

		# Sur les côtés 
	left_d0 = sum(data.ranges[80:90])/10
	right_d0 = sum(data.ranges[270:280])/10
	err0 = (right_d0 - left_d0)
		# Devant
	left_d1 = sum(data.ranges[50:60])/10
	right_d1 = sum(data.ranges[300:310])/10
	err1 = (right_d1 - left_d1)
		# Dérrière
	left_d2 = sum(data.ranges[110:120])/10
	right_d2 = sum(data.ranges[240:250])/10
	err2 = (right_d2 - left_d2)

	
	# Si la distance du robot au mur est supérieur à la safe_dist
	# c'est que il n'y a pas de mur devant et que la voie est libre.
	if d> safe_dist : 
		free_way = True
	else : 
		free_way = False 

	# Navigation laser 
	if commande_laser == True :
		sur_ligne = False 
		print 'Laser Navigation'

		# Arret d'urgence si le robot est dangereusement prés du mur 
		if test_b <0.3 :
			print 'Arret urgence : evitement mur' 
			# Si le mur est à droite le robot tourne à gauche
			if test1_b>test2_b :
				twist.angular.z = 0.4
				twist.linear.x = 0
			# Si le mur est à gauche le robot tourne à droite
			else : 
				twist.angular.z = -0.4
				twist.linear.x = 0

		# Si le robot ne détécte ni mur à gauche ni mur à droite il tourne sur lui même pour tenter de se repèrer 
		elif left_d1 >50 and right_d1>50 : 
			print 'mauvaise detection des murs' 
			twist.angular.z = 0.4
			twist.linear.x = 0
		
		# Si le robot ne detecte aucun mur en face de lui à droite
		elif left_d1 > 50 : 
			# Si le robot detecte qu'il est plus proche du mur gauche que droit sur les cotés
			# il tourne à droite 
			if 0<err0 : 
				twist.angular.z = - 0.5 
				twist.linear.x = lin_v
			# Situation inverse 
			elif 0>err0 : 
				twist.angular.z = 0.5 
				twist.linear.x = lin_v
			# Sinon le robot tente de detecter plus derrière en appliquant le même mécanisme 
			else : 	
				if 0<err2 : 
					twist.angular.z = - 0.5 
					twist.linear.x = lin_v
				elif 0>err2 : 
					twist.angular.z = 0.5 
					twist.linear.x = lin_v
				else :
					print 'pas de détection de mur à droite'
					twist.angular.z =  0.5
					twist.linear.x = 0
		# Si le robot ne detecte aucun mur en face de lui à gauche : même principe
		elif right_d1 > 50 : 
			if 0<err0 : 
				twist.angular.z = - 0.5 
				twist.linear.x = lin_v
			elif 0>err0 :
				twist.angular.z = 0.5 
				twist.linear.x = lin_v
			else : 	
				if 0<err2 : 
					twist.angular.z = - 0.5 
					twist.linear.x = lin_v
				elif 0>err2 :
					twist.angular.z = 0.5 
					twist.linear.x = lin_v
				else :
					print 'pas de détection de mur à gauche'
					twist.angular.z = - 0.5
					twist.linear.x = 0
		# Si le robot détecte qu'il est plus proche du mur gauche en face de lui : il tourne à droite
		elif 0<err1 : 
			twist.angular.z = - 0.5 
			twist.linear.x = lin_v
		# Si le robot détecte qu'il est plus proche du mur droite en face de lui : il tourne à gauche
		elif 0>err1 :
			twist.angular.z = 0.5 
			twist.linear.x = lin_v
		# Sinon il va tout droit
		else :
			twist.angular.z = 0
			twist.linear.x = lin_v

		# Commande de vitesse envoyé au robot 
		cmd_vel_pub.publish(twist)
################ Callback Scan Fin ##################################################################
	
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

	# Récupération du paramètre topic du scan, si pas de paramètre,
        #'/scan' = le nom le plus courant  
        if rospy.has_param('topic_scan'):
            topic_scan = rospy.get_param('topic_scan')
        else :
            topic_scan = '/scan'

	# Récupération du paramètres topic de la caméra, si pas de paramètre,
        #'/camera/image_raw' = par defaut   
        if rospy.has_param('topic_cam'):
            topic_cam = rospy.get_param('topic_cam')
        else :
            topic_cam = '/camera/image_raw'

	# Initialisation des variables globales 
	free_way = True
	laser_view = False
	sur_ligne = True 
	lost = False
	en_recherche = False 
	n_tours = 0
	nego_intersection = False

	# Récupération du paramètre de vitesse 'maximum' du robot 
	lin_v_max = rospy.get_param('/linear_scale')

	# Creation d'un node : 
	rospy.init_node('challenge3_task3_Circuit', anonymous=True)
	
	# Definition d'un publisher sur le topic de commande de vitesse du robot 
	cmd_vel_pub = rospy.Publisher(topic,Twist, queue_size=10)

	# Definition d'un publisher pour l'ouverture de porte 
	cmd_opendoor = rospy.Publisher('/Garage_Door_Opener',Bool, queue_size=10)

	# Definition d'un subscriber au topic du LIDAR 
	rospy.Subscriber(topic_scan, LaserScan, callback_scan)

	# Definition d'un subscriber au topic de la camera 
	rospy.Subscriber(topic_cam, Image, callback_cam)


	try:
        	# Attente d'arret du noeud ...
        	rospy.spin()

	except rospy.ROSInterruptException:
       		pass
