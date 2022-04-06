#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

class publisher_node:
    def __init__(self):
        self.pub = rospy.Publisher('robot_location', Point, queue_size= 10)
        self.rate = rospy.Rate(10) # 10Hz

        self.location = Point()
        self.location.y = 0
        self.location.z = 0
        self.location.x = 0

        while not rospy.is_shutdown():
            self.publish_and_update()
            self.rate.sleep()

    def publish_and_update(self):
        rospy.loginfo("Publishing location: {0}, {1}".format(self.location.x, self.location.y))
        self.pub.publish(self.location)

        self.location.x += 1
        self.location.y += 2

        self.rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('robot_location_publisher')
        pn = publisher_node()
    except rospy.ROSInterruptException:
        pass
