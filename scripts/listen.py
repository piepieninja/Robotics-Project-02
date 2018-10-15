#!/usr/bin/env python
import rospy
import numpy
import random
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

if __name__ == '__main__':
    x_map = numpy.zeros(100,)
    y_map = numpy.zeros(100,)
    x_map[0:10] = 0.1
    y_map[0:10] = 0.1
    #sensor_x = numpy.zeros(5)
    #sensor_y = numpy.zeros(5)
    sensor = numpy.zeros(5)
    sensor[2] = 0.6
    sensor[0:5:5] = 0.05
    sensor[1:4:3] = 0.15
    move_x = numpy.zeros(3,3)
    move_y = numpy.zeros(3,3)
    move_x[1,1] = 0.6
    move_x[0:3:2,1] = 0.05
    move_x[1,0:3:2] = 0.15
    move_y[1,1] = 0.6
    move_y[0:3:2,1] = 0.15
    move_y[1,0:3:2] = 0.05
