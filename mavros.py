import rospy
import cv2
from sys import argv
from mavros_msgs.srv import CommandBool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from mavros_msgs.srv import SetMode
rospy.init_node('computer_vision')
bridge = CvBridge()
img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
script, first = argv
if first == 'disarm':
    arming(False)
elif first == 'photo':
    cv2.imwrite('/var/www/CRTClover/photo.png', img)
elif first == 'setmode':
    set_mode(custom_mode='OFFBOARD')