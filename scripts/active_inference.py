#!/usr/bin/env python

###########################################################################
# Active Inference algorithm
#
# Execute the AI algorithm using the data from the 
# /filter/y_coloured_noise topic and publish the results to the 
# /filter/ai/output topic.
# Note that only the filtering part of the AI algorithm is implemented yet.
# 
# Author: Dennis Benders, TU Delft
# Last modified: 17.11.2019
# 
###########################################################################

#Import all necessary packages
import rospy                        #needed to be able to program in Python
import numpy as np                  #needed to be able to work with numpy
import time                         #needed to be able to get the execution time of code parts
from scipy.linalg import toeplitz   #needed to create derivative matrix in general way
from scipy.linalg import block_diag #needed to create the block-diagonal PI matrix

from jackal_active_inference_versus_kalman_filter.msg import gazebo_model_states_noise  #needed to read the custom output messages gazebo_model_states_noise
from jackal_active_inference_versus_kalman_filter.msg import filt_output                #needed to publish the custom output messages filt_output resulting from the filtering methods

#TODO:
#-finish the implementation with a correct usage of the learning rate, precision matrices and prior
#-implement the update rule for the next control input
#-extend the algorithm to work on all system model states
#-use IMU data in case of experiment with Jackal robot

#Active Inference class
#-------------------------------------------------------------------
class AI(object):
    """Class providing all AI functionality:
    - initialization of all necessary matrices
    - compute belief mu
    - compute control action u"""
    
    def __init__(self, n_states, n_inputs, n_outputs, p, x_ref):
        super(AI, self).__init__()
        
        #Input processing
        self.p = p
        
        #Indicating the first time AI function is called
        self.first_time = True
        
        #System dimensions
        self.n_states = n_states
        self.n_inputs = n_inputs
        self.n_outputs = n_outputs
        
        #Initial states
        self.x_0 = np.matrix(np.zeros(shape = (self.n_states, 1)))
        self.mu_0 = np.matrix(np.zeros(shape = ((1 + self.p) * self.n_states, 1)))
        self.mu = self.mu_0
        self.mu_dot = np.matrix(np.zeros(shape = ((1 + self.p) * self.n_states, 1)))
        
        #Initial system input (u) and output (z)
        self.u = np.matrix(np.zeros(shape = (self.n_inputs, 1)))
        self.z = np.matrix(np.zeros(shape = (self.n_outputs, 1)))
        
        #Derivative matrix
        self.Der = np.kron(np.eye((1 + self.p), k = 1), np.matrix(np.eye(self.n_states)))
        
        #Learning rates #TODO: tune these values when correct usage of precision matrices is known
        self.alpha_mu = 3.408*10**(-6)
#        self.alpha_u = 0.01
        
        #System matrices
        self.A = -209.6785884514270
        self.A_tilde = np.kron(np.eye(1 + self.p), self.A)
        self.B = np.matrix('16.921645797507500 -16.921645797507500')
        self.C = 1
        self.C_tilde = np.kron(np.matrix(np.eye(1 + self.p)), self.C)
        
        #Initial reference path (needed for prior belief): assuming no prior belief should be given
        self.x_ref = x_ref
        temp = np.matrix(np.zeros(shape = ((1 + self.p), 1)))
        temp[0] = 1
        self.mu_ref = np.kron(temp, self.x_ref)   #this assumes that reference acceleration of the robot will always be zero (the reference velocity constant)!
        self.xi = self.Der * self.mu_ref - self.A_tilde * self.mu_ref
        
        #Forward model #TODO: is this one always correct to use or should it actually be combined with alpha_u for update rule of u?
#        self.G = -1 * self.C * (1 / self.A) * self.B
    
    
    def construct_precision_matrices(self, sigma_w, s_w, sigma_z, s_z):
        '''Using the standard deviation information of the process output noise signals, construct the precision matrices'''
        
        #Process noise precision matrix
        self.sigma_w = sigma_w
        self.s_w = s_w
        self.SIGMA_w = np.matrix(np.eye(self.n_states)) * self.sigma_w**2
        self.PI_w = self.generate_PI(1 + self.p, self.SIGMA_w, self.s_w)
        
        #Output noise precision matrix
        self.sigma_z = sigma_z
        self.s_z = s_z
        self.SIGMA_z = np.matrix(np.eye(self.n_states)) * self.sigma_z**2
        self.PI_z = self.generate_PI(1 + self.p, self.SIGMA_z, self.s_z)
        
        #Total precision matrix
        self.PI = block_diag(self.PI_w, self.PI_z)


    def generate_PI(self, k, SIGMA, s):
        if np.amax(SIGMA) == 0:
            print("PI cannot be generated if sigma is 0 or negative")
            
        n = SIGMA.shape[0]
        
        if s != 0:
            l = np.array(range(0, 2*k-1, 2))
            rho = np.matrix(np.zeros(shape = (1, 2*k-1)))
            rho[0,l] = np.cumprod(1-l)/(np.sqrt(2)*s)**l
            
            V = np.matrix(np.zeros(shape = (k, k)))
            for r in range(k):
                V[r,:] = rho[0,r:r+k]
                rho = -rho
            
            SIGMA_tilde = np.kron(V, SIGMA)
            PI = np.linalg.inv(SIGMA_tilde)
            
        else:
            PI = np.matrix(np.zeros(shape = (k*n, k*n)))
            PI[0:n, 0:n] = np.linalg.inv(SIGMA)
        
        return PI


    def compute_mu(self):
        '''Update belief mu'''
        self.mu_dot = self.Der * self.mu - self.alpha_mu * ((self.Der - self.A_tilde).getT() * self.PI_w * (self.Der * self.mu - self.A_tilde * self.mu - self.xi) - self.C_tilde.getT() * self.PI_z * (self.z_gen - self.C_tilde * self.mu))
#        self.mu_dot = self.Der * self.mu - self.alpha_mu * ((self.Der - self.A_tilde).getT() * self.PI_w * (self.Der * self.mu - self.A_tilde * self.mu - self.xi) - self.C_tilde.getT() * self.PI_z * (self.z - self.C_tilde * self.mu))
        
        self.mu = self.mu + self.mu_dot * self.delta_t


    def compute_u(self):
        '''Update control action u'''
#        self.u_dot = -1 * self.alpha_u * self.G.getT() * self.PI_z * (self.z - self.C_tilde * self.mu)
#        self.u = self.u + self.u_dot * self.delta_t
        
        
    def debug(self):
        '''Debug function for AI functionality: print all kinds of desirable variables'''
        print("Der:\n{}\n\nmu:\n{}\n\nmu_dot:\n{}\n\nA_tilde:\n{}\n\nPI_w:\n{}\n\nxi:\n{}\n\nC_tilde:\n{}\n\nPI_z:\n{}\n\n-------------------------------------------------------------------------------------------\n".format(self.Der, self.mu, self.mu_dot, self.A_tilde, self.PI_w, self.xi, self.C_tilde, self.PI_z))
        
        print("Der*mu:\n{}\n\n2nd term:\n{}\n\n3rd term:\n{}\n\nmu_dot:\n{}\n\nmu:\n{}\n\n-------------------------------------------------------------------------------------------\n-------------------------------------------------------------------------------------------\n".format(self.Der*self.mu, self.alpha_mu * ((self.Der - self.A_tilde).getT() * self.PI_w * (self.Der * self.mu - self.A_tilde * self.mu - self.xi)), self.alpha_mu * (self.C_tilde.getT() * self.PI_z * (self.z - self.C_tilde * self.mu)), self.mu_dot, self.mu))

        print("C_tildeT:\n{}\n\nPI_z:\n{}\n\nC_tildeT*PI_z:\n{}\n\nz:\n{}\n\nC_tilde:\n{}\n\nC_tilde*mu:\n{}\n\nz-C_tilde*mu:\n{}\n\n-------------------------------------------------------------------------------------------\n".format(self.C_tilde.getT(), self.PI_z, self.C_tilde.getT()*self.PI_z, self.z, self.C_tilde, self.C_tilde*self.mu, self.z-self.C_tilde*self.mu))
        
        print("C_tildeT*PI_z:\n{}\n\nz:\n{}\n\nC_tilde*mu:\n{}\n\nz-C_tilde*mu:\n{}\n\n3rd term:\n{}\n\n-------------------------------------------------------------------------------------------\n-------------------------------------------------------------------------------------------\n".format(self.C_tilde.getT()*self.PI_z, z, self.C_tilde*self.mu, z-self.C_tilde*self.mu, self.C_tilde.getT() * self.PI_z * (z - self.C_tilde * self.mu)))
#-------------------------------------------------------------------


#Subscriber class
#-------------------------------------------------------------------
class Subscriber(object):
    """Class providing all functionality needed to:
    - subscribe to the measurement data
    - run the AI equations
    - publish the result"""
    def __init__(self):
        super(Subscriber, self).__init__()

        #Create AI object
        self.mean_u = np.matrix([[4.183917321479406], [1.942289357961973]])
        self.mean_y = 0.401988453296692
        self.debug = False
        self.n_states = 1
        self.p = 6
        self.x_ref = np.matrix(np.zeros(shape = (self.n_states, 1)))
        #---------------------------------------------
        self.ai = AI(n_states = self.n_states, n_inputs = 1, n_outputs = 1, p = self.p, x_ref = self.x_ref)

        #Initialize node, publisher and subscriber
        self.msg = filt_output()  #construct the custom message filt_output
        rospy.init_node('ai', anonymous=True)
        self.publisher = rospy.Publisher('filter/ai/output', filt_output, queue_size=1)
        rospy.Subscriber('filter/y_coloured_noise', gazebo_model_states_noise, self.callback)
        rospy.spin()
        
        
    def callback(self, data):
        '''Get system output z and call AI functionality'''
        #The first time data comes in, the Gazebo model states update time is known and the precision matrices can be constructed
        if self.ai.first_time:
            self.ai.delta_t = data.delta_t                  #get time difference between two subsequent Gazebo model states data updates
            self.ai.construct_precision_matrices(data.sigma_w, data.s_w, data.sigma_z, data.s_z)
            self.ai.first_time = False
            
        #Transform system output from operating point to origin and provide to AI algorithm
        self.z = data.y_model_noise[2]
        self.ai.z = self.z - self.mean_y
        temp = np.matrix(np.zeros(shape = (1 + self.p, 1)))
        temp[0,0] = 1
        self.ai.z_gen = np.kron(temp, self.ai.z)

        #Call AI functionality
        if self.debug:
            self.ai.debug()
        self.ai.compute_mu()
        self.ai.compute_u()
        self.x_filt = self.ai.mu[:self.n_states, 0] + 1/self.ai.C*self.mean_y

        #Publish result AI algorithm
        self.msg.x_filt = [float(self.x_filt)]
#        self.msg.u = [float(i) for i in self.ai.u]
#        self.msg.u_lin = []
#        for i,x in enumerate(self.msg.u):
#            self.msg.u_lin.append(x - self.mean_u[i])
        self.msg.y = [float(self.z)]
        self.msg.y_lin = [float(self.ai.z)]
        self.publisher.publish(self.msg)
#-------------------------------------------------------------------


#Main function
if __name__ == '__main__':
    subscriber = Subscriber()
