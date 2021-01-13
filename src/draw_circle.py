#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time
from circle_test import circle_IK

def draw_circle():

    # Initialize node
    rospy.init_node('draw_circle_node')

    while rospy.get_rostime().to_sec() == 0.0:
        time.sleep(0.1)
        print(rospy.get_rostime().to_sec())
    
    # Initiate publisher
    trajectory_pub = rospy.Publisher('/planar_robot/joint_trajectory_controller/command',
             JointTrajectory, queue_size=5)

    # Ensure publisher is connected
    rate = rospy.Rate(1)
    while trajectory_pub.get_num_connections() == 0:
        rate.sleep()
    
    # Create an instance of type JointTrajectory
    traj = JointTrajectory()

    # Set "traj" header
    traj.header.stamp = rospy.Time.now()
    traj.header.frame_id = "base_link"

    # Set "traj" joint names
    traj.joint_names.append("joint_1")
    traj.joint_names.append("joint_2")

    # Set the joint angles
    n = 100
    dt = 0.01
    radius = 0.45
    duration = 3

    for i in range(n):

        # Call the circle_IK function
        q, X, A = circle_IK(radius, duration, i*dt*duration)
        
        point = JointTrajectoryPoint()
        point.positions.append(q[0])
        point.positions.append(q[1])

        traj.points.append(point)

        traj.points[i].time_from_start = rospy.Duration.from_sec(i*dt*duration)
        rospy.loginfo("Status: joint_angles[%f][%f, %f]", i*dt*duration, q[0], q[1])
    
    trajectory_pub.publish(traj)
    rospy.spin()
    


if __name__ == "__main__":
    try:
        draw_circle()
    except rospy.ROSInterruptException:
        pass