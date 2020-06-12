#!/usr/bin/env python
# -*- coding: utf-8 -*-
#include <ros/console.h>

##############################################################################################################
# Code du noeud utilisé dans le challenge 2 Task 2. Ce noeud est lancé dans challenge2_task2.launch et demande au robot de garder une distance constante au mur en mouvement se trouvant en face de lui. 

# Le principe de base de ce code (publisher, subscriber ...) reste semblable au challenge 2 Task 1. Cepdendant la commande du robot n'est plus réalisé à partir de simples condtions mais d'un calcul de vitesse fonction de la distance au mur avec coéficients de type des système de contrôle correcteur PID (Proportionnel, Integral, Dérivé).
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

	# Distance du robot au mur : 
	d1 = data.ranges[0:5]
	d2 = data.ranges[355:360]
	d = (sum(d1) + sum(d2))/10

	# Dans le cas ou le robot est 'éloigné' du mur (marge = 30cm) : instruction lui est donnée d'avancer vers celui-ci 
	# (à vitesse fixée et raisonnable)
	# Cela permet de controller l'instabilitée du modèle PID implémenté plus bas en reduisant sont utilisation à une
	# distance proche de la distance désirée 
	if d > safe_dist + 0.3 : 
		twist.linear.x = 0.4

	# Sinon : activation du mantiens à distance avec le modèle correcteur PID
	else : 
		# Coeficients PID : 
		Ki = 0.04 
		Kp = 3.5 
		Kd = 3 
		# Commande vitesse à l'aide d'un PID 
		Last_Errp = Errp
		Errp = - (safe_dist + 0.20 - d) # l'ajout 0.20 compense l'erreure fixe entre distance réelle et souhaitée 
		I += Errp
		D = Errp - Last_Errp

		twist.linear.x = Kp * (Errp)+ Ki * I + Kd * D 

	# Envoi de la commande de vitesse 
	twist.angular.z = 0 # sécurité non obligatoire en théorie 
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
	
	# Création d'un node challenge2_task2_distanceFromWall : 
	rospy.init_node('challenge2_task2_distanceFromWall', anonymous=True)

	# Definition d'un publisher sur le topic de commande de vitesse du robot 
	cmd_vel_pub = rospy.Publisher(topic,Twist, queue_size=10)
	
	# Definition d'un subscriber au topic du LIDAR 
	rospy.Subscriber(topic_scan, LaserScan, callback)


	try:
        	# Attente d'arret du noeud ...
        	rospy.spin()

	except rospy.ROSInterruptException:
       		pass
