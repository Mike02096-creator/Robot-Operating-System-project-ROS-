#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import roslib
import numpy as np


from gazebo_msgs.msg import ModelState, ModelStates






if __name__ == '__main__':
	try:
		# Node name
		rospy.init_node('OBSTACLE3', anonymous=False)



		pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1, latch=True) #When a connection is latched, a reference to the last message published is saved and sent to any future subscribers that connect. This is useful for slow-changing or static data like a map. 


		# Random command to make models blocks one or the other way 
		rand=np.random.randint(4)

		current_state=ModelState()

		msg_block1=ModelState()
		msg_block1.model_name="block"	

		msg_block2=ModelState()
		msg_block2.model_name="block_clone"

		current_state=rospy.wait_for_message('/gazebo/model_states',ModelStates)

		ind1=current_state.name.index(msg_block1.model_name) 
		ind2=current_state.name.index(msg_block2.model_name) 



		msg_block1.pose=current_state.pose[ind1] # store the current position to set it in the next published message
		msg_block2.pose=current_state.pose[ind2] # store the current position to set it in the next published message

		if rand==0:
			print('0')	
			msg_block1.pose.position.y=0.97
			msg_block2.pose.position.y=0.67				

		elif rand==1:
			print('1')
			msg_block1.pose.position.y=5.37
			msg_block2.pose.position.y=0.67	
	

		elif rand==2:
			print('2')
			msg_block1.pose.position.y=0.97	
			msg_block2.pose.position.y=5.57		

		elif rand==3:
			print('3')
			msg_block1.pose.position.y=5.37
			msg_block2.pose.position.y=5.57		


		pub.publish(msg_block1)
		rospy.sleep(0.5)
		pub.publish(msg_block2)
		rospy.sleep(0.5)












		msg=ModelState()
		
		msg.model_name="obstacle3"
		current_state=rospy.wait_for_message('/gazebo/model_states',ModelStates)
		ind=current_state.name.index(msg.model_name) # Find the index of the model in the list of models published on the topic model_states
		rospy.sleep(1)

		vitesse=1
		while not rospy.is_shutdown():
			current_state=rospy.wait_for_message('/gazebo/model_states',ModelStates)
			msg.pose=current_state.pose[ind] # store the current position to set it in the next published message
			msg.pose.position.z=0
			pub.publish(msg)
			rospy.sleep(10+5*np.random.random_sample()) # random time between 10 and 15 s where the obstacle is on the road

			
			current_state=rospy.wait_for_message('/gazebo/model_states',ModelStates)
			msg.pose=current_state.pose[ind] # store the current position to set it in the next published message
			msg.pose.position.z=0.4
			pub.publish(msg)
			rospy.sleep(5) # 5s to go under the obstacle 



   
	except rospy.ROSInterruptException:


		pass