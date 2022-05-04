#!/usr/bin/env python

# import debugpy

# debugpy.listen(5678)
# debugpy.wait_for_client()

import rospy
import cv2
import imutils
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

class image_processing_node:
    def __init__(self, name):
        self.bridge = CvBridge()
        self.colour_frame = None
        self.depth_frame = None
        self.mask_frame = None
        self.num_colour_images = 0
        self.num_depth_images = 0

        self.ranges = [
            [(119, 241, 92), (122, 255, 245), 'blue'],
            [(59, 241, 92), (65, 255, 245), 'green'],
            [(0, 241, 92), (8, 255, 245), 'red'],
            [(27, 241, 92), (33, 255, 245), 'yellow']
        ]

        # use normal topic and 'Image' data type instead of compressed
        self.subscriber_colour = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_colour)
        self.subscriber_depth = rospy.Subscriber('/camera/depth/image_raw', Image, self.callback_depth)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()

    def callback_colour(self, colour_image):
        rospy.loginfo('[Image Processing] callback_colour')

        colour_contours = []

        try:
            self.colour_frame = self.bridge.imgmsg_to_cv2(colour_image, "bgr8")
        except CvBridgeError as e:
            print(e)

        blurred = cv2.GaussianBlur(self.colour_frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # b_min = (119, 241, 92)
        # b_max = (122, 255, 201)

        # g_min = (59, 241, 92)
        # g_max = (65, 255, 201)

        # r_min = (0, 241, 92)
        # r_max = (8, 255, 201)

        # y_min = (27, 241, 92)
        # y_max = (33, 255, 201)

        for limit in self.ranges:
            local_mask = cv2.inRange(hsv, limit[0], limit[1])
            local_mask = cv2.erode(local_mask, None, iterations=2)
            local_mask = cv2.dilate(local_mask, None, iterations=2)
            
            contours = cv2.findContours(local_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = imutils.grab_contours(contours)

            if len(contours) != 0:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)
                colour_contours.append([largest_contour, y, limit[2]])
                
        colour_contours.sort(key=lambda x:x[1])

        colour_masks = self.colour_frame
        for colour in colour_contours:
            x, y, w, h = cv2.boundingRect(colour[0])
            colour_masks = cv2.rectangle(colour_masks, (x, y), (x+w, y+h), (0,0,255), 2)

        self.mask_frame = colour_masks
        print 'Colours from the top: {}, {}'.format(colour_contours[0][2], colour_contours[1][2])
        
            

    def callback_depth(self, depth_image):
        rospy.loginfo('[Image Processing] callback_depth')
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

    def loop(self):
        if self.colour_frame != None and self.depth_frame != None:
            cv2.imshow('Colour Image', self.colour_frame)
            cv2.imshow('Depth Image', self.depth_frame)
            cv2.imshow('Masked Image', self.mask_frame)
            resp = cv2.waitKey(80)
            if resp == ord('c'):
                rospy.loginfo('[Image Processing] Saving colour')
                cv2.imwrite('colour_{}.png'.format(self.num_colour_images), self.colour_frame)
                self.num_colour_images += 1
            if resp == ord('d'):
                rospy.loginfo('[Image Processing] Saving depth')
                cv2.imwrite('depth_{}.png'.format(self.num_depth_images), self.depth_frame)
                self.num_depth_images += 1


if __name__ == '__main__':
    print "Starting ROS Image Processing Module"
    rospy.init_node('image_proessing_node', anonymous=True, log_level=rospy.DEBUG)
    ipn = image_processing_node('image_processing_node')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image Processing Module"

