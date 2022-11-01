#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
 
def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + " Received Data %s", data.data)
    return
     
def listener():
    rospy.init_node('botSub', anonymous=True)
 
    rospy.Subscriber("inData", String, callback)
 
    rospy.spin()
 
if __name__ == '__main__':
    listener()