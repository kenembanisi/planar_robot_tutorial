#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import time

def go_to_position():

    # Initialize node
    rospy.init_node('set_position_node')

    rate = rospy.Rate(1)
    
    # Initiate publisher
    joint_1_pub = rospy.Publisher('/planar_robot/joint_1_position_controller/command',
             Float64, queue_size=1)

    joint_2_pub = rospy.Publisher('/planar_robot/joint_2_position_controller/command',
             Float64, queue_size=1)

    # Set the command values
    j1 = Float64()
    j2 = Float64()
    j1.data = 1.57
    j2.data = 0
    
    while joint_1_pub.get_num_connections() == 0 or joint_2_pub.get_num_connections() == 0:
        rate.sleep()

    print("Publishing to joint controller")

    joint_1_pub.publish(j1)
    joint_2_pub.publish(j2)

    rospy.spin()
    


if __name__ == "__main__":
    try:
        go_to_position()
    except rospy.ROSInterruptException:
        pass