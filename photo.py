#!/usr/bin/python3
import cv2
import rospy
rospy.init_node('computer_vision_sample')
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
bridge = CvBridge()
img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
cv2.imwrite('/var/www/CRTClover/photo.png', img)
