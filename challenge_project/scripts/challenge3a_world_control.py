#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import roslib
import numpy as np

from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelState, ModelStates

flag_door=0

def garage_callback(Garage_Door_Signal):
	vitesse=0.1
	global flag_door
	print(Garage_Door_Signal.data)

	if Garage_Door_Signal.data==True:
		if flag_door==0:
			rospy.loginfo("Open garage door")
			current_state=rospy.wait_for_message('/gazebo/model_states',ModelStates)
			msg.pose=current_state.pose[ind] # store the current position to set it in the next published message
			msg.twist.linear.z=vitesse
			pub.publish(msg)
			rospy.sleep(0.4/vitesse)
			current_state=rospy.wait_for_message('/gazebo/model_states',ModelStates)
			msg.pose=current_state.pose[ind] # store the current position to set it in the next published message
			msg.twist.linear.z=0.0
			pub.publish(msg)
			flag_door=1


	else:
		if flag_door==1:
			rospy.loginfo("Close garage door")
			current_state=rospy.wait_for_message('/gazebo/model_states',ModelStates)
			msg.pose=current_state.pose[ind] # store the current position to set it in the next published message
			msg.twist.linear.z=-vitesse
			pub.publish(msg)
			rospy.sleep(0.4/vitesse)
			current_state=rospy.wait_for_message('/gazebo/model_states',ModelStates)
			msg.pose=current_state.pose[ind] # store the current position to set it in the next published message
			msg.twist.linear.z=0.0
			pub.publish(msg)
			flag_door=0





if __name__ == '__main__':
	try:
		# Node name
		rospy.init_node('GARAGE_DOOR', anonymous=False)
		sub=rospy.Subscriber('/Garage_Door_Opener',Bool,garage_callback)
		pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1, latch=True) #When a connection is latched, a reference to the last message published is saved and sent to any future subscribers that connect. This is useful for slow-changing or static data like a map. 
		msg=ModelState()
		current_state=ModelState()
		msg.model_name="door"
		current_state=rospy.wait_for_message('/gazebo/model_states',ModelStates)
		ind=current_state.name.index(msg.model_name) # Find the index of the model in the list of models published on the topic model_states
		rospy.sleep(1)


		vitesse=1
		while not rospy.is_shutdown():
			a=1



   
	except rospy.ROSInterruptException:


		pass