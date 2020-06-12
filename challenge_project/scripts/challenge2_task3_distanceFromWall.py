#!/usr/bin/env python
# -*- coding: utf-8 -*-
#include <ros/console.h>

##############################################################################################################
# Code du noeud utilisé dans le challenge 2 Task 3. Ce noeud est lancé dans challenge2_task3.launch et demande au robot de rester en face et à distance constance d'un mur pouvant surgir partout autours de lui. 

# Le principe de maintiens à distance reste semblable à celui utilisé en challenge 2 Task 2 . 
# De plus il a été rajouté un mécanisme permettant au robot de se tourner en face du mur dès ce dernier détécté par le LIDAR. Le fonctionnement repose sur le fait que les données du lidar sont accessibles sous la forme de distance détécté pour chaque dégré autours du robot (de 0 à 359, 0 étant le devant du robot, 90 sa gauche et ainsi de suite), la distance la plus faible est alors concidérée comme indiquant l'angle ou se trouve le mur par rapport au robot.
# Les rotations sont fonctions de la valeur d'angle à couvrir pour se remettre droit.     
##############################################################################################################

# Importation des librairies utiles :
import rospy 

# Importation des types de messages traités ici 
from geometry_msgs.msg import Twist # Type de message pour le topic de commande de vitesse 
from sensor_msgs.msg import LaserScan # Type de message pour le topic du scan 

# Création d'un message de type Twist destiné à stocker les commandes de vitesses du robot 
twist=Twist()

########## Fonction callback ratachée au subscriber du topic du scanner 
def callback(data):

	# Variable globale qui représente l'erreure proportionnelle (distance souhaitée - distance réelle )
	global Errp
	# Variable globale qui représente l'intégrale des erreures proportionnelles au cours de la simulation
	global I 

	# Récupération du paramètre de distance de sécurité   
        if rospy.has_param('safety_dist'):
            safe_dist = rospy.get_param('safety_dist')
        else :
            safe_dist = 0.2 # valeur defaut fixée à 20 cm

	# Recherche position du mur (recherche de l'angle avec la distance la plus faible détéctée) :
	tabval = data.ranges 
	dist = min(tabval)
	ind = [i for i, x in enumerate(tabval) if x == dist]
	angle_deg = sum(ind)/len(ind) # moyenne : préviens l'éventualité de plusieurs distances minimum détéctées 
	
	# Definition des coéficients de rotation 
	Kr = 0.04 # Cas ou le mur n'est pas en face
	Kr2 = 0.015 # Cas où le mur est "presque" en face

	# Dans le cas ou le mur est détécté presque en face du robot (utilisation de Kr2)
	if 7<angle_deg or angle_deg > 353 : 
		if angle_deg <= 180 : 
			twist.angular.z = Kr2 *angle_deg 
		else : 
			twist.angular.z = Kr2 * (-(360 - angle_deg))

		#### Processus de maintient à distance du mur activé 
		#Distance du robot au mur : 
		d1 = data.ranges[0:5]
		d2 = data.ranges[355:360]
		d = (sum(d1) + sum(d2))/10
	
		if d > safe_dist + 0.3 : 
			twist.linear.x = 0.4
		else : 
			# Coeficients PID : 
			Ki = 0.04 
			Kp = 3.5 
			Kd = 3 
			# Commande vitesse à l'aide d'un PID 
			Last_Errp = Errp
			Errp = - (safe_dist + 0.2 - d) # l'ajout 0.20 compense l'erreure fixe entre distance réelle et souhaitée
			I += Errp
			D = Errp - Last_Errp

			twist.linear.x = Kp * (Errp)+ Ki * I + Kd * D
		#### Fin du processus de maintient à distance du mur
	
	# Si le mur est détécté trop à droite du robot :
	elif angle_deg <= 180 : 
		twist.angular.z = Kr *angle_deg 

	# Si le mur est détécté trop à gauche du robot :
	else : 
		twist.angular.z = Kr * (-(360 - angle_deg))

	# Envoi de la commande de vitesse
	cmd_vel_pub.publish(twist)
########## Fonction callback : Fin

if __name__ == '__main__':

	# Récupération du paramètre de distance de sécurité 
        # safe_dist = 0.2 par defaut  
        if rospy.has_param('safety_dist'):
            safe_dist = rospy.get_param('safety_dist')
        else :
            safe_dist = 0.2

	# Récupération du paramètre topic de la commande vitesse, si pas de paramètre,
        # '/cmd_vel' = le nom le plus courant  
        if rospy.has_param('topic_velocity'):
            topic = rospy.get_param('topic_velocity')
        else :
            topic = '/cmd_vel'

	# Récupération du paramètre topic du scan, si pas de paramètre,
        # '/scan' = le nom le plus courant  
        if rospy.has_param('topic_scan'):
            topic_scan = rospy.get_param('topic_scan')
        else :
            topic_scan = '/scan'

	# Initilaisation des variables globales 
	Errp = 0
	I = 0

	# Création d'un node challenge2_task3_distanceFromWall : 
	rospy.init_node('challenge2_task3_distanceFromWall', anonymous=True)
	
	# Definition d'un publisher sur le topic de commande de vitesse du robot 
	cmd_vel_pub = rospy.Publisher(topic,Twist, queue_size=10)
	
	# Definition d'un subscriber au topic du LIDAR 
	rospy.Subscriber(topic_scan, LaserScan, callback)


	try:
        	# Attente d'arret du noeud ...
        	rospy.spin()

	except rospy.ROSInterruptException:
       		pass
