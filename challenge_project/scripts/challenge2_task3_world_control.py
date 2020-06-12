#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf

from gazebo_msgs.msg import ModelState, ModelStates

if __name__ == '__main__':
    try:
        # Node name
        rospy.init_node('gazebo_models_animation', anonymous=False)

        # Lets define a publisher on the topic_name topic (Twist message)
        # When a connection is latched, a reference to the last message
        # published is saved and sent to any future subscribers that connect. 
        # This is useful for slow-changing or static data like a map. 
        pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1, latch=True)
        msg = ModelState()
        msg2 = ModelState()
        msg.model_name = "obstacle"
        pub.publish(msg)
        rospy.sleep(1)

        state_init = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        # Find the index of the model in the list of models published on the topic model_states
        ind = state_init.name.index(msg.model_name) 
        print(state_init)
        velocity = 0.2

        while not rospy.is_shutdown():
            msg.pose.orientation.x = 0
            msg.pose.orientation.y = 0
            msg.pose.orientation.z = 0
            msg.pose.orientation.w = 1
            msg.pose.position.x = 0
            msg.pose.position.y = 0
            msg.pose.position.z = 0
            pub.publish(msg)

            angle_pos_random = np.random.random_sample() * 2 * np.pi
            quaternion = tf.transformations.quaternion_from_euler(0,0, angle_pos_random)

            msg.pose.orientation.x = quaternion[0]
            msg.pose.orientation.y = quaternion[1]
            msg.pose.orientation.z = quaternion[2]
            msg.pose.orientation.w = quaternion[3]
            pub.publish(msg)

            rospy.loginfo("MOVING CLOSER")
            msg.twist.linear.x = velocity * np.cos(angle_pos_random)
            msg.twist.linear.y = velocity * np.sin(angle_pos_random)
            pub.publish(msg)
            rospy.sleep(4.0 / velocity)

            rospy.loginfo("MOVING AWAY")
            current_state = rospy.wait_for_message('/gazebo/model_states',ModelStates)
            # store the current position to set it in the next published message
            msg.pose.position = current_state.pose[ind].position 

            msg.twist.linear.x = -velocity * np.cos(angle_pos_random)
            msg.twist.linear.y = -velocity * np.sin(angle_pos_random)
            pub.publish(msg)
            rospy.sleep(2.3 / velocity)

    except rospy.ROSInterruptException:
        pass
