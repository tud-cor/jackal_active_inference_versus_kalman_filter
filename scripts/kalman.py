#!/usr/bin/env python

###########################################################################
# Kalman filter
#
# Execute the Kalman filter using the data from the 
# /filter/y_coloured_noise and /joint_states topic and publish the results 
# to the /filter/kalman/output topic.
# 
# Author: Dennis Benders, TU Delft
# Last modified: 17.11.2019
# 
###########################################################################

#Import all necessary packages
import rospy        #needed to be able to program in Python
import numpy as np  #needed to be able to work with numpy

from sensor_msgs.msg import JointState                  #needed to interpret the joint_state topic and derive the wheel speeds from it
from jackal_active_inference_versus_kalman_filter.msg import gazebo_model_states_noise      #needed to read the custom output messages gazebo_model_states_noise
from jackal_active_inference_versus_kalman_filter.msg import filt_output                    #needed to publish the custom output messages filt_output resulting from the filtering methods


#TODO:
#-extend the algorithm to work on all system model states
#-use IMU data in case of experiment with Jackal robot

#Kalman class
#-------------------------------------------------------------------
class Kalman(object):
    """Class providing all Kalman filter functionality:
    - initialization of all necessary matrices
    - Kalman update rule
    - Kalman one-step-ahead predication rule
    Derived from: https://en.wikipedia.org/wiki/Kalman_filter"""
    
    def __init__(self, n_states, n_inputs, n_outputs):
        super(Kalman, self).__init__()
        
        #Indicating the first time Kalman function is called
        self.first_time = True
        
        #System dimensions
        self.n_states = n_states
        self.n_inputs = n_inputs
        self.n_outputs = n_outputs

        #Kalman predictors
        self.x_k_k = 0      #initial state prediction
        self.P_k_k = 0      #initial error covariance matrix
        
        #System matrices - use discrete system matrices!!
        self.A = 0.810844818981278
        self.B = np.matrix('0.015265349684018 -0.015265349684018')
        self.C = 1
        
        #System input and output as provided by the subscribers
        self.u_k_1 = np.matrix(np.zeros(shape = (self.n_inputs, 1)))
        self.y_k = np.matrix(np.zeros(shape = (self.n_outputs, 1)))
        
        #Noise covariance matrices
        self.Q = np.matrix(np.eye(self.n_states))   #will be adjusted by using the sigma information of the process white noise signal
        self.R = np.matrix(np.eye(self.n_outputs))  #will be adjusted by using the sigma information of the output white noise signal


    def predict(self):
        '''One-step-ahead prediction'''
        #Update time
        self.x_k_1_k_1 = self.x_k_k
        self.P_k_1_k_1 = self.P_k_k
        
        #State prediction
        self.x_k_k_1 = self.A * self.x_k_1_k_1 + self.B * self.u_k_1
        
        #Covariance matrix prediction
        self.P_k_k_1 = self.A * self.P_k_1_k_1 * self.A + self.Q


    def update(self):
        '''Update Kalman variables to current time step'''
        #Innovation residual
        self.y_tilde = self.y_k - self.C * self.x_k_k_1
        
        #Innovation covariance
        self.S = self.C * self.P_k_k_1 * self.C + self.R
        
        #Optimal Kalman gain
        self.K = self.P_k_k_1 * self.C * (1 / self.S)
        
        #Updated state estimate
        self.x_k_k = self.x_k_k_1 + self.K * self.y_tilde
        
        #Updated covariance estimate
        self.P_k_k = (1 - self.K * self.C) * self.P_k_k_1
        
        #Measurement residual
#        self.y_tilde = self.y_k - self.C * self.x_k_k
        
        
    def debug(self):
        '''Debug function for Kalman functionality: print all kinds of desirable variables'''
        print("u_k_1:\n{}\n\ny_k:\n{}\n\n-----------------------------------------------------------------------------------------\n".format(self.u_k_1, self.y_k))
        print("x_k_1_k_1:\n{}\n\nx_k_k_1:\n{}\n\nP_k_1_k_1:\n{}\n\nP_k_k_1:\n{}\n\n-----------------------------------------------------------------------------------------\n".format(self.x_k_1_k_1, self.x_k_k_1, self.P_k_1_k_1, self.P_k_k_1))
        print("y_tilde:\n{}\n\nS:\n{}\n\nK:\n{}\n\nx_k_k:\n{}\n\nP_k_k:\n{}\n\n-----------------------------------------------------------------------------------------\n\n-----------------------------------------------------------------------------------------\n".format(self.y_tilde, self.S, self.K, self.x_k_k, self.P_k_k))
#-------------------------------------------------------------------


#Subscriber class
#-------------------------------------------------------------------
class Subscriber(object):
    """Class providing all functionality needed to:
    - subscribe to the measurement data
    - run the Kalman filter equations
    - publish the result"""
    def __init__(self):
        super(Subscriber, self).__init__()
        
        #Indications whether system input (u) or output (y) updates are available
        self.u_present = False
        self.y_present = False
        
        #Create Kalman object
        self.mean_u = np.matrix([[4.183917321479406], [1.942289357961973]])
        self.mean_y = 0.401988453296692
        self.debug = False
        self.kalman = Kalman(n_states = 1, n_inputs = 1, n_outputs = 1)

        #Initialize node, publisher and subscriber
        self.msg = filt_output()  #construct the custom message filt_output
        rospy.init_node('kalman', anonymous=True)
        self.publisher = rospy.Publisher('filter/kalman/output', filt_output, queue_size = 1)
        rospy.Subscriber('filter/y_coloured_noise', gazebo_model_states_noise, self.callback_y)
        rospy.Subscriber('joint_states', JointState, self.callback_u)
        rospy.spin()
        
        
    def callback_y(self, data):
        '''Get system output y''' 
        #Indicate that a new measurement has been received
        self.y_present = True
        
        #The first time data comes in, the Q and R matrices can be constructed
        if self.kalman.first_time:
            self.kalman.Q *= data.sigma_w**2    #construct process noise covariance matrix using standard deviation of white process noise
            self.kalman.R *= data.sigma_z**2    #construct output noise covariance matrix using standard deviation of white output noise
            self.kalman.first_time = False
        
        #Convert input (output of system) to proper format
        self.y = np.matrix([[data.y_model[2]], [data.y_noise[2]], [data.y_model_noise[2]]])
        
        #Transform system output from operating point to origin and provide to Kalman filter
        self.y_lin = self.y - self.mean_y
        self.kalman.y_k = self.y_lin[2]

        #In case the system input update has also been received: run the Kalman filter for the next iteration
        if self.u_present:
            self.run_kalman()

    
    def callback_u(self, data):
        '''Get system input u'''
        #Indicate that a the system input update has been received
        self.u_present = True
        
        #Calculate average wheel rotational speeds on each side of the Jackal
        omega_left = (data.velocity[0] + data.velocity[2])/2
        omega_right = (data.velocity[1] + data.velocity[3])/2
        self.u = np.matrix([[omega_right], [omega_left]])
        
        #Transform system input from operating point to origin and provide to Kalman filter
        self.kalman.u_k_1 = self.u - self.mean_u
        
        #In case the system output update has also been received: run the Kalman filter for the next iteration
        if self.y_present:
            self.run_kalman()
        
        
    def run_kalman(self):
        '''Run Kalman filter functionality and publish results to topic filter/kalman/output'''
        #Reset the indication variables
        self.u_present = False
        self.y_present = False
        
        #Run Kalman filter for one iteration
        self.kalman.predict()
        self.kalman.update()
        if self.debug:
            self.kalman.debug()
            
        #Transform estimated system state back to operating point
        self.x_filt = self.kalman.x_k_k + 1/self.kalman.C*self.mean_y

        #Publish result Kalman filter
        self.msg.x_filt = [float(self.x_filt)]
        self.msg.u = [float(i) for i in self.u]
        self.msg.u_lin = [float(i) for i in self.kalman.u_k_1]
        self.msg.y = [float(i) for i in self.y]
        self.msg.y_lin = [float(i) for i in self.y_lin]
        self.publisher.publish(self.msg)
#-------------------------------------------------------------------


#Main function
if __name__ == '__main__':
    subscriber = Subscriber()
