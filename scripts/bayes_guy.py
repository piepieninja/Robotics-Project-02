#!/usr/bin/env python
import rospy
import random
import message_filters
import numpy as np
import matplotlib.pyplot as plt
# imports
from PIL import Image
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
from Robotics_Project_02.msg import Sensor_range
from Robotics_Project_02.msg import Motion
from filterpy.discrete_bayes import normalize
from filterpy.discrete_bayes import update
from filterpy.discrete_bayes import predict

likelihood_x = np.zeros(100, dtype=float)
likelihood_y = np.zeros(100, dtype=float)
#likelihood_x[2] = 1.0
#likelihood_y[2] = 1.0
prior_x = np.zeros(100, dtype=float)
prior_y = np.zeros(100, dtype=float)
prior_x[2] = 1.0
prior_y[2] = 1.0
posterior_x = np.zeros(100, dtype=float)
posterior_y = np.zeros(100, dtype=float)
posterior_x[2] = 1.0
posterior_y[2] = 1.0
#check if Twist is the correct message type
#pub = rospy.Publisher("/BAYES_OUTPUT", Twist, queue_size=10) 
gaussian_values = [0.0, 0.04779, 0.904419, 0.04779, 0.0]
kernel = [.1, .8, .1]
prevX = 2
prevY = 2
lastPoseX = 2
lastPoseY = 2

def obs_model(x, y, axis):
    global prevX
    global prevY
    global prior_x
    global prior_y
    global posterior_x
    global posterior_y
    likelihood_x = np.zeros(100, dtype=float)
    g_i = 0;
    for i in range(x-2,100):
        if g_i == 5:
            break
        if i < 0:
	  continue
        likelihood_x[i] = gaussian_values[g_i]
        g_i += 1
    likelihood_y = np.zeros(100, dtype=float)
    #print 'likelihood_x:'
    #print likelihood_x
    g_i = 0;
    for i in range(x-2,100):
        if g_i == 5:
            break
	if i < 0:
	  continue
        likelihood_y[i] = gaussian_values[g_i]
        g_i += 1
    #print 'likelihood_y:'
    #print likelihood_y
    if axis=="x":
        prior_x = predict(posterior_x,x-prevX,gaussian_values)
        posterior_x = update(likelihood_x, prior_x)
    if axis=="y":
        prior_y = predict(posterior_y,y-prevY,gaussian_values)
        posterior_y = update(likelihood_y, prior_y)
    #print 'post x:'
    #print posterior_x
    #print 'post y:'
    #print posterior_y
    
    prevX = x
    prevY = y

def motion_model_update(axis, distance):
    global lastPoseX
    global lastPoseY
    global prior_x
    global prior_y
    global posterior_x
    global posterior_y
    space = 0
    xOffset = 0
    yOffset = 0
    likelihood_x = np.zeros(100, dtype=float)
    g_i = 0;
    if axis == "x":
      xOffset = distance
      space = 2 * distance
      for i in range(lastPoseX+distance-1, 100):
        if g_i == 3:
          break
        if i < 0:
	  continue
        likelihood_x[i] = kernel[g_i]
        g_i += 1
    #print 'likelihood_x:'
    #print likelihood_x
    
    space = 0
    likelihood_y = np.zeros(100, dtype=float)
    g_i = 0;
    if axis == "y":
      yOffset = distance
      space = 2 * distance
      for i in range(lastPoseY+distance-1, 100):
          if g_i == 3:
              break
	  if i < 0:
	    continue
          likelihood_y[i] = kernel[g_i]
          g_i += 1
    #print 'likelihood_y:'
    #print likelihood_y

    if axis=="x":
      prior_x = predict(posterior_x,xOffset,kernel)
      posterior_x = update(likelihood_x, prior_x)  
    if axis=="y":
      prior_y = predict(posterior_y,yOffset,kernel)  
      posterior_y = update(likelihood_y, prior_y)
    #print 'post x:'
    #print posterior_x
    #print 'post y:'
    #print posterior_y
    lastPoseX += xOffset
    lastPoseY += yOffset

def heatmap_viewer():
    print "generating heatmap..."
    world_mat = np.matmul(posterior_x.transpose(), posterior_y)
    img = Image.new( 'RGB', (100,100), "black") # create a new black image
    pixels = img.load() # create the pixel map
    for x in range(0,100):
            for y in range(0,100):
                color = int(round(255 * world_mat[x,y])) 
                if color > 255:
                    color = 255
                pixels[x,y] = (color,100,100)
    #plt.imshow(world_mat, cmap='hot', interpolation='nearest')
    #plt.show()
    img.show()
    img.save("current_step.png")
    
    
def callback(sensor_data, control_data):
    print "yeet: "
    motion_model_update(control_data.axis,control_data.dist)
    obs_model(int(round(sensor_data.z1)),int(round(sensor_data.z2)),control_data.axis)
    # get and print 5x5 max region in x
    max = 0.0
    max_i = -1
    for x in range(0,100):
        if (posterior_x[x] > max):
            max = posterior_x[x]
            max_i = x
    print "X max " + str(max) + " at index: " + str(max_i)
    temp = []	
    for x in range (max_i-3,max_i+4):
        temp.append(posterior_x[x])
    print str(temp)
    # now print y
    max = 0.0
    max_i = -1
    for y in range(0,100):
        if (posterior_y[y] > max):
            max = posterior_y[y]
            max_i = y
    print "Y max " + str(max) + " at index: " + str(max_i)
    temp = []	
    for y in range (max_i-3,max_i+4):
        temp.append(posterior_y[y])
    print str(temp)
    heatmap_viewer()
    # end printing out
    
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
