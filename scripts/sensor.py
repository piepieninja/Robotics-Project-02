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
    print data.pose.pose
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    # do something
    mu = 0.0
    sigma = 1.0
    x_range = x + random.gauss(mu, sigma)
    y_range = y + random.gauss(mu, sigma)
    print "x + gaussian: " + str(x_range) + ", y + gaussian: " + str(y_range) 
    # publish values to the guy
    msg = Sensor_range
    msg.z1 = x_range
    msg.z2 = y_range
    pub = rospy.Publisher('sensor_range', Sensor_range, queue_size=10)
    pub.publish(msg)

    
def listener():
    print "pls"
    rospy.init_node('sensor_x', anonymous=True)    
    rospy.Subscriber("base_pose_ground_truth", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
