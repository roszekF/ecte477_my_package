#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class MoveTB3():
    def __init__(self):
        self.tb3_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)

    def publish_once_in_cmd_vel(self):
        while not self.ctrl_c:
            connections = self.tb3_vel_publisher.get_num_connections()
            if connections > 0:
                self.tb3_vel_publisher.publish(self.cmd)
                rospy.loginfo('Cmd published')
                break
            else:
                self.rate.sleep()
    
    def shutdownhook(self):
        self.ctrl_c = True

    def move_tb3(self, moving_time=5.0, linear_speed=0.5, angular_speed=0.0):
        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed
        
        rospy.loginfo("Moving TB3")
        i = 0
        while not self.ctrl_c and i <= moving_time:
            self.publish_once_in_cmd_vel()
            i += 1
            self.rate.sleep()

        self.stop_tb3()

    def stop_tb3(self):

        self.cmd.linear.x = 0
        self.cmd.angular.z = 0

        rospy.loginfo('Stopping TB3')
        self.publish_once_in_cmd_vel()

if __name__ == '__main__':
    rospy.init_node('move_tb3_proper', anonymous=True)
    movetb3_obj = MoveTB3()
    try:
        movetb3_obj.move_tb3()
    except rospy.ROSInterruptException:
        pass