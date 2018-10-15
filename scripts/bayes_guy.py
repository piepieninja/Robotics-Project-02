#!/usr/bin/env python
import rospy
import random
import message_filters
import numpy as np
# imports
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
from Robotics_Project_02.msg import Sensor_range
from Robotics_Project_02.msg import Motion
from filterpy.discrete_bayes import normalize
from filterpy.discrete_bayes import update
from filterpy.discrete_bayes import predict

likelihood_x = np.zeros(100)
likelihood_y = np.zeros(100)
likelihood_x[2] = 1
likelihood_y[2] = 1
prior_x = np.zeros(100)
prior_y = np.zeros(100)
prior_x[2] = 1
prior_y[2] = 1
posterior_x = np.zeros(100)
posterior_y = np.zeros(100)
#check if Twist is the correct message type
#pub = rospy.Publisher("/BAYES_OUTPUT", Twist, queue_size=10) 
gaussian_values = [.06, .24, 0.4, .24, .06]
kernel = [.1, .8, .1]

def obs_update(x, y):
    likelihood_x = np.zeros(100)
    likelihood_x[x] = gaussian_values[2]
    if x == 99:    
        likelihood_x[x-2] = gaussian_values[0]
        likelihood_x[x-1] = gaussian_values[1]
    elif x == 98:
        likelihood_x[x-1] = gaussian_values[1]
        likelihood_x[x-2] = gaussian_values[0]
        likelihood_x[x+1] = gaussian_values[3]
    elif x == 0:
        likelihood_x[x+1] = gaussian_values[3]
        likelihood_x[x+2] = gaussian_values[4]
    elif x == 1:
        likelihood_x[x+1] = gaussian_values[3]
        likelihood_x[x+2] = gaussian_values[4]     
        likelihood_x[x-1] = gaussian_values[1]
    else:
        likelihood_x[x+1] = gaussian_values[3]
        likelihood_x[x+2] = gaussian_values[4]     
        likelihood_x[x-1] = gaussian_values[1]     
        likelihood_x[x-2] = gaussian_values[0]
    posterior_x = update(likelihood_x, prior_x)
  
    likelihood_y = np.zeros(100)
    likelihood_y[y] = gaussian_values[2]
    if x == 99:    
        likelihood_y[y-2] = gaussian_values[0]
        likelihood_y[y-1] = gaussian_values[1]
    elif x == 98:
        likelihood_y[y-1] = gaussian_values[1]
        likelihood_y[y-2] = gaussian_values[0]
        likelihood_y[y+1] = gaussian_values[3]
    elif x == 0:
        likelihood_y[y+1] = gaussian_values[3]
        likelihood_y[y+2] = gaussian_values[4]
    elif x == 1:
        likelihood_y[y+1] = gaussian_values[3]
        likelihood_y[y+2] = gaussian_values[4]     
        likelihood_y[y-1] = gaussian_values[1]
    else:
        likelihood_y[y+1] = gaussian_values[3]
        likelihood_y[y+2] = gaussian_values[4]     
        likelihood_y[y-1] = gaussian_values[1]     
        likelihood_y[y-2] = gaussian_values[0]
    posterior_y = update(likelihood_y, prior_y)

def motion_model_update(axis, distance):
    if axis == "x":
        prior_x = predict(posterior_x, distance, kernel)
    if axis == "y":
        prior_y = predict(posterior_y, distance, kernel)

def callback(sensor_data, control_data):
    print "yeet: "
    #print sensor_data
    #print odom_data
    #print control_data
    x = sensor_data.z1
    y = sensor_data.z2
    motion_model_update(control_data.axis,control_data.dist)
    obs_model(x,y)
    # get and print 5x5 max region in x
    max = 0
    max_i = -1
    for x in range(0,100):
        if (posterior_x[x] > max):
            max = posterior_x[x]
            max_i = x
    print "X max " + str(max) + " at index: " + str(max_i)
    temp = []	
    for x in range (max_i-3,max_i+3):
        temp.append(posterior_x[x])
    print str(temp)
	#for i in posterior_x:
    #    if posterior_x[i] != 0:
    #        print(
    
def listener():
    print "pls"
    rospy.init_node('bayes_guy', anonymous=True)
    
    sensor_sub  = message_filters.Subscriber("sensor_range", Sensor_range)
    #odom_sub    = message_filters.Subscriber("base_odometry/odom", Odometry)
    control_sub = message_filters.Subscriber("motion_model", Motion)
    #changed timeSynch to approximateTimeSynch so as to allow bayes guy to capture messages in 0.3 sec interval and ones with no headers(command topic). 
    ts = message_filters.ApproximateTimeSynchronizer([sensor_sub, control_sub], 10, 0.3, allow_headerless=True)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
