#!/usr/bin/env python
import rospy
import random
# imports
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
from Robotics_Project_02.msg import Sensor_range

x_limit = 100
y_limit = 100
sensor_limit = 50

def sense(true_x,true_y):
    # calculate the distances w/ the sensor model
    mu = 0.0
    sigma = 0.4
    x_range = (true_x) + random.gauss(mu, sigma)
    y_range = (true_y) + random.gauss(mu, sigma)
    if (x_range > sensor_limit):
        x_range = sensor_limit
    if (y_range > sensor_limit):
        y_range = sensor_limit
    # make the message type and fill it
    ranges = Sensor_range()
    ranges.z1 = x_range
    ranges.z2 = y_range
    # yeeto complete-o
    return ranges

def callback(data):
    print data.pose.pose
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    msg = sense(x,y)
    msg.header = data.header
    print msg
    pub = rospy.Publisher('sensor_range', Sensor_range, queue_size=10)
    pub.publish(msg)

    
def listener():
    print "pls"
    rospy.init_node('sensor', anonymous=True)    
    rospy.Subscriber("base_pose_ground_truth", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
