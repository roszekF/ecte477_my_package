#!/usr/bin/env python

# import debugpy

# debugpy.listen(5678)
# debugpy.wait_for_client()

import rospy
import cv2
import imutils
import numpy as np
import transformations as trans
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from nav_msgs.msg import Odometry
from visualization_msgs import MarkerArray, Marker
from geometry_msgs.msg import Quaternion
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
            [(119, 241, 92), (122, 255, 245), 'blue', (0,0,255,255)],
            [(59, 241, 92), (65, 255, 245), 'green', (0,255,0,255)],
            [(0, 241, 92), (8, 255, 245), 'red', (255,0,0,255)],
            [(27, 241, 92), (33, 255, 245), 'yellow', (127,127,0,255)]
        ]

        self.K = None
        self.transform_cam_to_world = None

        self.marker_array = MarkerArray()

        # use normal topic and 'Image' data type instead of compressed
        self.subscriber_colour = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_colour)
        self.subscriber_depth = rospy.Subscriber('/camera/depth/image_raw', Image, self.callback_depth)

        self.subscriber_camera_info = rospy.Subscriber('camera/rgb/camera_info', CameraInfo, self.callback_camera_info)
        self.subscriber_odometry = rospy.Subscriber('odom', Odometry, self.callback_odometry)

        # self.publisher_markers = rospy.Publisher(...)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            self.find_colors()
            r.sleep()

    def find_colors(self):
        if self.colour_frame == None:
            return

        colour_contours = []

        frame = self.colour_frame
        # prepare the frame for tresholding
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # b_min = (119, 241, 92)
        # b_max = (122, 255, 201)

        # g_min = (59, 241, 92)
        # g_max = (65, 255, 201)

        # r_min = (0, 241, 92)
        # r_max = (8, 255, 201)

        # y_min = (27, 241, 92)
        # y_max = (33, 255, 201)

        # loop through the color treshold ranges
        for limit in self.ranges:
            local_mask = cv2.inRange(hsv, limit[0], limit[1])
            local_mask = cv2.erode(local_mask, None, iterations=2)
            local_mask = cv2.dilate(local_mask, None, iterations=2)
            
            contours = cv2.findContours(local_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = imutils.grab_contours(contours)

            # check if there are any contours
            if len(contours) != 0:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)

                # append the largest ('nearest') contour to the array
                # along with it's Y coordinate, color name, and center point and RGBA color value
                colour_contours.append([largest_contour, y, limit[2], (x+h/2, y+w/2)], limit[3])
                
        # check if only one beacon is in front
        if len(colour_contours) != 2:
            return
        # sort the contours array by the Y coordinate (top contour first)
        colour_contours.sort(key=lambda x:x[1])

        colour_masks = self.colour_frame

        for colour in colour_contours:
            x, y, w, h = cv2.boundingRect(colour[0])
            # draw rectangle arraund the colour contour
            colour_masks = cv2.rectangle(colour_masks, (x, y), (x+w, y+h), (0,0,255), 2)
            # draw a circle at the center
            colour_masks = cv2.circle(colour_masks, colour[3], radius=10, color=(0,255,0), thickness=-1)

        self.mask_frame = colour_masks
        print 'Colours from the top: {}(x:{}, y:{}), {}'.format(colour_contours[0][2], colour_contours[0][3][0], colour_contours[0][3][1], colour_contours[1][2])

        if self.K == None or self.transform_cam_to_world == None or self.depth_frame == None:
            return
        
        # x, y of the top colour contour
        x, y = colour_contours[0][3]

        depth = self.depth_frame[x, y]
        p_h = np.array([[x], [y], [1]])
        p3d = depth * np.matmul(np.linalg.inv(self.K), p_h)
        p3d_h = np.array([[p3d[2][0]], [-p3d[0][0]], [-p3d[1][0]], [1]])
        p3d_w_h = np.matmul(self.transform_cam_to_world, p3d_h)
        # result is an array of [x, y, z]
        p3d_w = np.array([[p3d_w_h[0][0]/p3d_w_h[3][0]], [p3d_w_h[1][0]/p3d_w_h[3][0]], [p3d_w_h[2][0]/p3d_w_h[3][0]]])

        marker = Marker()
        # marker.header.seq= marker.id = 
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.pose = Pose(Point(p3d_w[0], p3d_w[1], 1), Quaternion(0.0, 1.0, 0.0, 1.0))
        marker.scale = Vector3(2,2,2)
        marker.color = ColorRGBA(colour_contours[0][4])

        self.marker_array.markers.append(marker)
        self.publisher_markers.publish(self.marker_array)


    def callback_colour(self, colour_image):
        # rospy.loginfo('[Image Processing] callback_colour')
        # convert the image to cv2 format
        try:
            self.colour_frame = self.bridge.imgmsg_to_cv2(colour_image, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_depth(self, depth_image):
        # rospy.loginfo('[Image Processing] callback_depth')
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

    def callback_camera_info(self, camera_info):
        self.K = np.array(camera_info.K).reshape([3, 3])

    def callback_odometry(self, odometry):
        self.transform_cam_to_world = trans.msg_to_se3(odometry.pose.pose)

    def loop(self):
        if self.mask_frame != None and self.depth_frame != None:
            # cv2.imshow('Colour Image', self.colour_frame)
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

