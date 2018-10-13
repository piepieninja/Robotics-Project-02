#!/usr/bin/env python
import rospy
import random
import message_filters
# imports
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
from Robotics_Project_02.msg import Sensor_range

def callback(sensor_data,odom_data,control_data):
    print "yeet: "
    print sensor_data
    print odom_data
    print control_data
    x = sensor_data.z1
    y = sensor_data.z2

    
def listener():
    print "pls"
    rospy.init_node('bayes_guy', anonymous=True)
    
    sensor_sub  = message_filters.Subscriber('sensor_range', Sensor_range)
    odom_sub    = message_filters.Subscriber('base_odometry/odom', Odometry)
    control_sub = message_filters.Subscriber('base_controller/command', Twist)
    
    ts = message_filters.TimeSynchronizer([sensor_sub,odom_sub,control_sub], 10)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
