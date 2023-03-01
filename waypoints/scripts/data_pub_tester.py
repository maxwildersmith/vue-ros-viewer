#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose

def get_q(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

start = [47.362947,8.541062,0]

pose = Pose()
pose.position.x = start[0]
pose.position.y = start[1]
pose.position.z = start[2]
pose.orientation.w = 1

rospy.init_node('sample_data')
rate = rospy.Rate(1)
pub = rospy.Publisher('/position', Pose, queue_size=10)

def update():
    pose.position.x = start[0] + 0.001*np.sin(rospy.get_time()/30)
    pose.position.y = start[1] + 0.001*np.cos(rospy.get_time()/30)
    pose.position.z =  start[2] + -10*(np.cos(rospy.get_time()/50)+1)
    r = 0.5*np.sin(rospy.get_time()/60)
    p = 0.8*np.cos(rospy.get_time()/100)
    y = 0.2*np.sin(rospy.get_time()/50)
    q = get_q(r,p,y)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

try:
    while not rospy.is_shutdown():
        rospy.loginfo('Sample moving')
        pub.publish(pose)
        update()
        rate.sleep()

except Exception as e:
    rospy.logerr(e)
    pass
