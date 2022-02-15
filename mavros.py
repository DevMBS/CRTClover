import rospy
from sys import argv
script, first = argv
if first == 'disarm':
    from mavros_msgs.srv import CommandBool
    arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    arming(False)
elif first == 'photo':
    import cv2
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    rospy.init_node('computer_vision')
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    cv2.imwrite('/var/www/CRTClover/photo.png', img)