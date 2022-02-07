#!/usr/bin/python3
import rospy
from mavros_msgs.srv import CommandBool
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
arming(False)
