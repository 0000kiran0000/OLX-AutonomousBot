#!/usr/bin/env python3

from math import cos, sin
import rospy
import socket
import time
import json
import roslib
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Imu, PointCloud
from geometry_msgs.msg import Vector3, Quaternion, Point32
import tf


inpVals = dict({"uwb": 0.0, "imu":{"a":[0.0, 0.0, 0.0], "g":[0.0, 0.0, 0.0]}, "cmp":0.0})
controlVals = dict({"th": 1500.0, "st": 75.0})


imuDataHandle = rospy.Publisher('sensor_msgs/Imu', Imu, queue_size=10)
bcnDataHandle = rospy.Publisher('olx_bot/bcn_dist', PointCloud, queue_size=10)



botTheta = 0.0

rospy.init_node('botPub', anonymous=True)
rospy.loginfo("Starting botPub Node")



SOCKET_BUFFER_SIZE = 1024

ImuData = Imu()
bcnData = PointCloud()
bcnData.points.append(Point32())


raspiSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def raspiConnect():
    global raspiSocket, raspiClient

    while True:
        rospy.sleep(1)
        try:
            raspiSocket.bind(('192.168.137.1', 6674))
            
            raspiSocket.listen(1)
            
            raspiClient,addr = raspiSocket.accept()
            

        except Exception as e:
            rospy.loginfo(e)
        
        rospy.loginfo("jAMES!")

        return
        


if __name__ == '__main__': 
    
    raspiConnect()
    
    while not rospy.is_shutdown():
        try:
            #inpVals = (raspiClient.recv(1024).decode())
            inpVals = json.loads(str(raspiClient.recv(1024).decode()))
            #rospy.loginfo("Err a")
            raspiClient.send(bytes((str(json.dumps(controlVals))).encode("utf-8")))
            #rospy.loginfo("Err b")
            #rospy.loginfo(inpVals)
            #rospy.loginfo(type(inpVals['bcnDist']))
            


            #tfBroadcaster.sendTransform((0,0,0), tf.transformations.quaternion_from_euler(float(inpVals['compass']['a'][2]), 90.0 - float(inpVals['compass']['a'][2]), 0.0), rospy.Time.now(), 'olxbot', "world")
            #tfBroadcaster.sendTransform((0,0,0), tf.transformations.quaternion_from_euler(0.0, 0.0, float(inpVals['compass']['a'][2])), rospy.Time.now(), 'olxbot', "world")

            

            ImuData.header.stamp = rospy.get_rostime()
            ImuData.header.frame_id = "world"
            
            orientationQuat = tf.transformations.quaternion_from_euler(0.0, 0.0, float(inpVals['cmp']))

            print(float(inpVals['cmp']))
            ImuData.orientation.x = orientationQuat[0]
            ImuData.orientation.y = orientationQuat[1]
            ImuData.orientation.z = orientationQuat[2]
            ImuData.orientation.w = orientationQuat[3]
            #ImuData.orientation_covariance[0] = 0.0
            #ImuData.orientation_covariance[4] = 0.0
            #ImuData.orientation_covariance[8] = float(inpVals['compass']['a'][2]) * float(inpVals['compass']['a'][2])	

            ImuData.angular_velocity.x = inpVals['imu']['g'][0]
            ImuData.angular_velocity.y = inpVals['imu']['g'][1]
            ImuData.angular_velocity.z = inpVals['imu']['g'][2]
            #ImuData.angular_velocity_covariance[0] = 0.0
            #ImuData.angular_velocity_covariance[4] = 0.0
            #ImuData.angular_velocity_covariance[8] = 0.0

            ImuData.linear_acceleration.x = inpVals['imu']['a'][0]
            ImuData.linear_acceleration.y = inpVals['imu']['a'][1]
            ImuData.linear_acceleration.z = inpVals['imu']['a'][2]
            #ImuData.linear_acceleration_covariance[0] = 0.0
            #ImuData.linear_acceleration_covariance[4] = 0.0
            #ImuData.linear_acceleration_covariance[8] = 0.0
            #print(type(ImuData))
            

            bcnData.header.stamp = rospy.get_rostime()
            bcnData.header.frame_id = "world"
            r = float(inpVals['uwb'])
            
            
            
            bcnData.points[0] = Point32(r*cos(botTheta), r*sin(botTheta), 0.0)

            #bcnData.points[0].x = r*cos(botTheta)
            #bcnData.points[0].y = r*sin(botTheta)
            #bcnData.points[0].z = 0.0
            botTheta += 0.1


            imuDataHandle.publish(ImuData)
            
            bcnDataHandle.publish(bcnData)

            
            

            #rospy.sleep(0.1)

        except Exception as e:
            print(str(e))
            time.sleep(2)
            #raspiConnect()

    raspiSocket.close()
    #raspiSocket.shutdown(socket.SHUT_RDWR)
