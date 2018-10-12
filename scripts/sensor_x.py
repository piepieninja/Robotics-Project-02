#!/usr/bin/env python
import rospy
import random
# imports
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64

def callback(data):
    print "yeet: "
    print data.pose.pose
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    # do something
    mu = 0.0
    sigma = 1.0
    x_range = x + random.gauss(mu, sigma)
    # publish values to the guy
    pub = rospy.Publisher('sensor_x_range', Float64, queue_size=10)
    pub.publish(x_range)

    
def listener():
    print "pls"
    rospy.init_node('sensor_x', anonymous=True)    
    rospy.Subscriber("base_pose_ground_truth", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
