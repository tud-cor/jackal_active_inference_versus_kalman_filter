#!/usr/bin/env python

###########################################################################
# Simulation control
#
# Publish the system input to the /cmd_vel topic at a frequency of 50 Hz, 
# except when a keyboard key is pressed. Using the keyboard, the robot 
# motion can be adjusted a bit.
# 
# Author: Dennis Benders, TU Delft
# Last modified: 17.11.2019
# 
###########################################################################

#Import all necessary packages
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import numpy as np
import time


#Global variables for keeping track of keyboard activity and time
key = False
prev_time = 0.0


#Function called every time a new message is received on the cmd_vel topic
#Used to determine whether the robot is automatically controlled using a pre-defined motion
#or the keyboard (teleop) is controlling the robot 
def callback(data):
    global prev_time
    #First time handling of setting prev_time
    if prev_time == 0.0:
        prev_time = time.time()
    #In the rest of the cases: calculate the publishing rate on this topic
    else:
        cur_time = time.time()
        publish_rate = 1.0/(cur_time - prev_time)
        prev_time = cur_time
        #When continuously pressing a key, the rate is certainly above 50
        if publish_rate > 52.0:
            key = True
        #Publishing rate is slower than continuously pressing a key
        else:
            key = False


#Create a subscriber to the velocity topic
def move_subscribe():
    rospy.Subscriber('cmd_vel', Twist, callback)


#Create a publisher and publish the pre-defined robot motion
def move_publish():
    #Set up publisher    
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('motion_control', anonymous=True)
    rate = rospy.Rate(50) # default: 50Hz
    #Define constant motion
    v_deg = 22.5
    v_rad = v_deg/180*np.pi
    linear_control = Vector3(0.3, 0.0, 0.0) #m/s
    angular_control = Vector3(0.0, 0.0, v_rad) #rad/s
    #Publish loop
    while not rospy.is_shutdown():
        rate.sleep()
        #Always publish the pre-defined motion, unless a key is continuously pressed
        if not key:
            message = Twist(linear_control, angular_control)
            publisher.publish(message)


#Main function
if __name__ == '__main__':
    try:
        move_subscribe()        
        move_publish()
    except rospy.ROSInterruptException:
        pass

