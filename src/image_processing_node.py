#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

class image_processing_node:
    def __init__(self, name):
        self.bridge = CvBridge()
        self.colour_frame = None
        self.num_colour_images = 0

        # use normal topic and 'Image' data type instead of compressed
        self.subscriber_colour = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_colour)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()

    def callback_colour(self, colour_image):
        rospy.loginfo('[Image Processing] callback_colour')
        # from the lab:
        # np_array = np.fromstring(colour_image.data, np.uint8)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(colour_image, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        # from the lab:
        # self.colour_frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
        self.colour_frame = cv_image

    def loop(self):
        if self.colour_frame != None:
            cv2.imshow('Colour Image', self.colour_frame)
            resp = cv2.waitKey(80)
            if resp == ord('c'):
                rospy.loginfo('[Image Processing] Saving colour')
                cv2.imwrite('colour_{}.png'.format(self.num_colour_images), self.colour_frame)
                self.num_colour_images += 1


if __name__ == '__main__':
    print "Starting ROS Image Processing Module"
    rospy.init_node('image_proessing_node', anonymous=True, log_level=rospy.DEBUG)
    ipn = image_processing_node('image_processing_node')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image Processing Module"

