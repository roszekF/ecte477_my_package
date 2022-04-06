#!/usr/bin/env python

from publisher_node import publisher_node
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MoveTB3_untill_1m():
    def __init__(self):
        self.sub = rospy.Subscriber('/scan', LaserScan, self.sub_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(1)
        self.cmd = Twist()
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def publish_once(self):
        while not self.ctrl_c:
            connections = self.pub.get_num_connections()
            if connections > 0:
                self.pub.publish(self.cmd)
                rospy.loginfo('Cmd published')
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        self.ctrl_c = True

    def move_tb3(self):
        self.cmd.linear.x = 0.5

        rospy.loginfo('Moving TB3')
        self.publish_once()

    def stop_tb3(self):
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0

        rospy.loginfo('Stopping TB3')
        self.publish_once()

    def sub_callback(self, msg):
        rospy.loginfo('distance in front: {0}'.format(msg.ranges[0]))
        
        if msg.ranges[0] <= 1:
            rospy.loginfo('Wall detected')
            self.stop_tb3()
            rospy.signal_shutdown('Done')
            

if __name__ == '__main__':
    rospy.init_node('move_tb3_proper', anonymous=True)
    movetb3_obj = MoveTB3_untill_1m()
    try:
        movetb3_obj.move_tb3()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass