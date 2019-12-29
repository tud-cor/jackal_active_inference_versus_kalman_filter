#!/usr/bin/env python

###########################################################################
# Data processing
#
# Generate coloured output noise, add it to the Gazebo model states and 
# publish the results to the /filter/y_coloured_noise topic.
# 
# Author: Dennis Benders, TU Delft
# Last modified: 17.11.2019
# 
###########################################################################

#Import all necessary packages
import rospy        #needed to be able to program in Python
import numpy as np  #needed to be able to work with numpy

from gazebo_msgs.msg import ModelStates                 #definition of model states data coming from Gazebo simulation
from jackal_active_inference_versus_kalman_filter.msg import gazebo_model_states_noise      #needed to publish the custom output messages gazebo_model_states_noise resulting from adding colored noise to the Gazebo model states data

#TODO:
#-s_w and s_z are now only based on data of phi_dot, do this for other states to be able to use state estimation for all states in AI and Kalman filters
#-read IMU data for doing the experiment with the Jackal robot (not the Gazebo simulation)

#Data_processing class
#-------------------------------------------------------------------
class Data_processing(object):
    """Class providing all functionality needed to:
    - generate colored output noise
    - subscribe to the Gazebo model states data
    - add the noise to the simulation data
    - publish the result"""
    def __init__(self):
        super(Data_processing, self).__init__()
        
        #Ensure proper timing
        self.index = 0                                  #used to take the correct noise sample from the pre-generated array
        self.T = 17                                     #total noise generating time in s
        self.dt = 0.001                                 #1 kHz Gazebo model states data update
        
        #Create standard deviations
        self.sigma_w = 0.0086532635                     #standard deviation of white process noise, determined for phi_dot (see analyze_process_noise.m)
        self.s_w = 0                                    #standard deviation of filter for creating coloured process noise
        self.sigma_z = 5*self.sigma_w                   #standard deviation of white output noise
        self.s_z = 0.1                                  #standard deviation of filter for creating coloured output noise
        
        #Pre-calculate the coloured output signal to be added to the Gazebo data
        self.generate_coloured_output_noise()
        
        #Initialize gazebo_model_states_noise message
        self.msg = gazebo_model_states_noise()          #construct the custom message gazebo_model_states_noise
        self.msg.delta_t = self.dt                      #1 kHz Gazebo model states data update
        self.msg.sigma_w = self.sigma_w
        self.msg.s_w = self.s_w
        self.msg.sigma_z = self.sigma_z
        self.msg.s_z = self.s_z

        #Initialize node, publisher and subscriber
        rospy.init_node('Gazebo_model_states_listener', anonymous=True)
        self.publisher = rospy.Publisher('filter/y_coloured_noise', gazebo_model_states_noise, queue_size=1)
        rospy.Subscriber("gazebo/model_states", ModelStates, self.callback)
        rospy.spin()
        
        
    def generate_coloured_output_noise(self):
        '''Generate a white noise signal with specified standard deviation and create a coloured noise signal of specified length by filtering with a Gaussian filter with a specified kernel width'''
        #Derived from: "Derivation of generalised covariance matrix", written by Martijn Wisse
        #Determine the length of the noise signal
        N = int(self.T/self.dt + 1)                     #total amount of noise samples
        t = np.linspace(0,self.T,N)
        
        #Create three white noise signals for output
        np.random.seed(1)
        omega_z1 = np.random.randn(N)*self.sigma_z      #white noise signal used to construct noise for linear velocity in x direction (output)
        np.random.seed(2)
        omega_z2 = np.random.randn(N)*self.sigma_z      #white noise signal used to construct noise for linear velocity in y direction (output)
        np.random.seed(3)
        omega_z3 = np.random.randn(N)*self.sigma_z      #white noise signal used to construct noise for angular velocity around z-axis (output)
        
        #Create the Gaussian filter for process and output
        tau = np.linspace(-self.T,self.T,2*N-1)
        if self.s_z is not 0:
            h_z = np.sqrt(self.dt/(self.s_z*np.sqrt(np.pi))) * \
            np.exp(-(tau)**2/(2*self.s_z**2))                  #Gaussian filter impulse response for output
            
        #Create colored noise signal by taking the convolution between the Gaussian filter and the white noise signals
        if self.s_z is not 0:
            self.z1 = np.convolve(omega_z1, h_z, mode = 'valid')    #colored noise signal to be added to the linear velocity in x direction (output)
            self.z2 = np.convolve(omega_z2, h_z, mode = 'valid')    #colored noise signal to be added to the linear velocity in y direction (output)
            self.z3 = np.convolve(omega_z3, h_z, mode = 'valid')    #colored noise signal to be added to the angular velocity around z-axis (output)
        else:
            self.z1 = omega_z1
            self.z2 = omega_z2
            self.z3 = omega_z3
        self.noise_len = len(self.z1)
       
        
    def callback(self, data):
        '''Get Gazebo model states data, add noise and publish result'''
        #Get Gazebo model states data
        self.msg.y_model = np.array([data.twist[1].linear.x, data.twist[1].linear.y, data.twist[1].angular.z])
        
        #Get correct noise sample, scaled with factor f
        if self.index < self.noise_len:
            self.msg.y_noise = np.array([self.z1[self.index], self.z2[self.index], self.z3[self.index]])
            self.index += 1
        else:
            self.msg.y_noise = np.array([0, 0, 0])

        #Add colored output noise to Gazebo model states data
        if self.index < self.noise_len:
            self.msg.y_model_noise = self.msg.y_model + self.msg.y_noise
        else:
            self.msg.y_model_noise = np.array([0, 0, 0])

        #Publish result: original Gazebo model states data, colored noise and noisy Gazebo model states data
        self.publisher.publish(self.msg)
#-------------------------------------------------------------------


#Main function
if __name__ == '__main__':
    data_processing = Data_processing()
