#!/usr/bin/env python
import rospy
# imports
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String

def callback(data):
    print "yeet: "
    print data.pose.pose
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    z = odom.pose.pose.position.z
    # do something ...
    # ...
    # profit?
    
    
    
def listener():
    print "pls"
    rospy.init_node('sensor_x', anonymous=True)    
    rospy.Subscriber("base_pose_ground_truth", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
