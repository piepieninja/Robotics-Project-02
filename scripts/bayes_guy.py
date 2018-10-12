#!/usr/bin/env python
import rospy
import random
# imports
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
from Robotics_Project_02.msg import Sensor_range

def callback(data):
    print "yeet: "
    print "z1: " + str(data.z1) + ", z2: " + str(data.z2)
    # add in the randomness
    
def listener():
    print "pls"
    rospy.init_node('bayes_guy', anonymous=True)    
    rospy.Subscriber("sensor_range", Sensor_range, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
