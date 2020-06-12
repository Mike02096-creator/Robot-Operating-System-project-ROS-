#!/usr/bin/env python
# -*- coding: utf-8 -*-
#include <ros/console.h>

##############################################################################################################
# Code du noeud utilisé dans le challenge 2 Task 1. Ce noeud est lancé dans challenge2_task1.launch et demande au robot d'avancer vers un mur se trouvant en face de lui puis de s'arrèter lorsque que la distance de sécuritée préalablement définie est atteinte. 

# Pour cela nous utilisons les informations fournies par le laser (LIDAR) du robot : definition d'un subscriber au topic du scan fournissant un message de type LaserScan (sensor_msgs.msg). 
# Le informations sont ensuites traités, à l'aide de conditions simples. 
# La definition d'un publisher sur le topic comportant les instructions de vitesse du robot permet ensuite de commander l'avancé ou l'arret du robot. 
##############################################################################################################

# Importation des librairies utiles :
import rospy 

# Importation des types de messages traités ici 
from geometry_msgs.msg import Twist # Type de message pour le topic de commande de vitesse 
from sensor_msgs.msg import LaserScan # Type de message pour le topic du scan 

# Création d'un message de type Twist destiné à stocker les commandes de vitesses du robot 
twist=Twist()

########## Fonction callback ratachée au subscriber du topic du scanner :
def callback(data):

	# Distance du robot au mur en face de lui : 
	d1 = data.ranges[0:5]
	d2 = data.ranges[355:360]
	d = (sum(d1) + sum(d2))/10
	print 'Distance au mur :', d
	
	# Dans le cas ou la distance détéctée est inférieure ou égale à la distance de sécurité
	# le robot s'arrète
	# l'ajout de 0.2 permet de compenser l'erreure fixe quand à la distance extimée par le radar
	if d<=safe_dist + 0.20 :
		 
		twist.linear.x = 0
		cmd_vel_pub.publish(twist)
	# Sinon l'ordre est donné au robot d'avancer vers le mur 
	else : 
		twist.linear.x = lin_v 
		cmd_vel_pub.publish(twist)
########## Fonction callback : Fin	
	

if __name__ == '__main__':

	# Récupération du paramètre de distance de sécurité 
        # safe_dist = 1 par defaut  
        if rospy.has_param('safety_dist'):
            safe_dist = rospy.get_param('safety_dist')
        else :
            safe_dist = 1

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

	# Récupération des paramètres de vitesse du robot 
	lin_v = rospy.get_param('/linear_scale')

	# Création d'un node challenge2_task1_distanceFromWall : 
	rospy.init_node('challenge2_task1_distanceFromWall', anonymous=True)
	
	# Definition d'un publisher sur le topic de commande de vitesse du robot 
	cmd_vel_pub = rospy.Publisher(topic,Twist, queue_size=10)
	
	# Definition d'un subscriber au topic du LIDAR 
	rospy.Subscriber(topic_scan, LaserScan, callback)


	try:
        	# Attente d'arret du noeud ...
        	rospy.spin()

	except rospy.ROSInterruptException:
       		pass
