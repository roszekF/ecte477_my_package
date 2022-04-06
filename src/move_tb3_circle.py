#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_turtlebot3_node')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)
move = Twist()
move.linear.x = 0.5
move.angular.z = 1

i=0
while i < 20:
    pub.publish(move)
    i=i+1
    rate.sleep()

while not rospy.is_shutdown():
    connections = pub.get_num_connections()
    if connections > 0:
        move.linear.x = 0.0
        move.angular.z = 0.0
        pub.publish(move)
        rospy.loginfo("Cmd published")
        break
    else:
        rate.sleep()
