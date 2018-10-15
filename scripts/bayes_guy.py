#!/usr/bin/env python
import rospy
import random
import message_filters
# imports
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
from Robotics_Project_02.msg import Sensor_range
from filterpy.discrete_bayes import normalize
from filterpy.discrete_bayes import update
from filterpy.discrete_bayes import predict


likelihood_x = numpy.zeros(100)
likelihood_y = numpy.zeros(100)
likelihood_x[2] = 1
likelihood_y[2] = 1
prior_x = numpy.zeros(100)
prior_y = numpy.zeros(100)
prior_x[2] = 1
prior_y[2] = 1
posterior_x = numpy.zeros(100)
posterior_y = numpy.zeros(100)
#check if Twist is the correct message type
#pub = rospy.Publisher("/BAYES_OUTPUT", Twist, queue_size=10) 
gaussian_values = [.06, .24, 0.4, .24, .06]
kernel = (.1, .8, .1)

def obs_update(x, y):
    likelihood_x = numpy.zeros(100)
    likelihood_x[x] = gaussian_values[2]
    if x = 99:    
        likelihood_x[x-2] = gaussian_values[0]
        likelihood_x[x-1] = gaussian_values[1]
    elif x = 98:
        likelihood_x[x-1] = gaussian_values[1]
        likelihood_x[x-2] = gaussian_values[0]
        likelihood_x[x+1] = gaussian_values[3]
    elif x = 0:
        likelihood_x[x+1] = gaussian_values[3]
        likelihood_x[x+2] = gaussian_values[4]
    elif x = 1:
        likelihood_x[x+1] = gaussian_values[3]
        likelihood_x[x+2] = gaussian_values[4]     
        likelihood_x[x-1] = gaussian_values[1]
    else:
        likelihood_x[x+1] = gaussian_values[3]
        likelihood_x[x+2] = gaussian_values[4]     
        likelihood_x[x-1] = gaussian_values[1]     
        likelihood_x[x-2] = gaussian_values[0]
    posterior_x = update(likelihood_x, prior_x)
  
    likelihood_y = numpy.zeros(100)
    likelihood_y[y] = gaussian_values[2]
    if x = 99:    
        likelihood_y[y-2] = gaussian_values[0]
        likelihood_y[y-1] = gaussian_values[1]
    elif x = 98:
        likelihood_y[y-1] = gaussian_values[1]
        likelihood_y[y-2] = gaussian_values[0]
        likelihood_y[y+1] = gaussian_values[3]
    elif x = 0:
        likelihood_y[y+1] = gaussian_values[3]
        likelihood_y[y+2] = gaussian_values[4]
    elif x = 1:
        likelihood_y[y+1] = gaussian_values[3]
        likelihood_y[y+2] = gaussian_values[4]     
        likelihood_y[y-1] = gaussian_values[1]
    else:
        likelihood_y[y+1] = gaussian_values[3]
        likelihood_y[y+2] = gaussian_values[4]     
        likelihood_y[y-1] = gaussian_values[1]     
        likelihood_y[y-2] = gaussian_values[0]
    posterior_y = update(likelihood_y, prior_y)

def motion_model_update():


def callback(sensor_data,odom_data, control_data):
    print "yeet: "
    print sensor_data
    print odom_data
    print control_data
    x = sensor_data.z1
    y = sensor_data.z2

    
def listener():
    print "pls"
    rospy.init_node('bayes_guy', anonymous=True)
    
    sensor_sub  = message_filters.Subscriber("sensor_range", Sensor_range)
    odom_sub    = message_filters.Subscriber("base_odometry/odom", Odometry)
    control_sub = message_filters.Subscriber("base_controller/command", Twist)
    #changed timeSynch to approximateTimeSynch so as to allow bayes guy to capture messages in 0.3 sec interval and ones with no headers(command topic). 
    ts = message_filters.ApproximateTimeSynchronizer([sensor_sub,odom_sub, control_sub], 10, 0.3, allow_headerless=True)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
