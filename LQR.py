#!/usr/bin/env python
"""
Created on Thu Feb 16 13:27:16 2017

@author: AMROY_GAOL
"""

from __future__ import print_function, division

import rospy 
import controlpy
import numpy as np
from ReadData import uav
from plant import matrix
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty 

class Lqr():
    def __init__(self):
        rospy.init_node('LQR', anonymous=False)
        self.rate = rospy.Rate(20)
        self.K = np.matrix([[0]])
        self.sys = np.matrix([[0]])
        self.feedBack=np.matrix([[0]])
        
        self.desire = np.matrix([[0],
                                 [0],
                                 [0],
                                 [0],
                                 [0],
                                 [0],
                                 [0],
                                 [0]])
                                 
        self.x = np.matrix([[uav.roll],
                            [uav.roll_dot],
                            [uav.pitch],
                            [uav.pitch_dot],
                            [uav.yaw],
                            [uav.yaw_dot],
                            [uav.Z],
                            [uav.Z_dot]])
                            
        self.pubCommand = rospy.Publisher('cmd_vel',Twist, queue_size=10)
        self.pubLand = rospy.Publisher("ardrone/land",Empty, queue_size=10)
        self.pubTakeoff = rospy.Publisher("ardrone/takeoff",Empty, queue_size=10)
        self.command = Twist()   
        rospy.on_shutdown(self.SendLand)     
    def SetGain(self):
        matrix.VarMatrix()
        matrix.SetMatrix()
        #menghitung gain K, persamaan riccati X, dan eigen value matriks
        self.K, X, closedLoopEigVals = controlpy.synthesis.controller_lqr(matrix.A, matrix.B, matrix.Q, matrix.R)
        #print(closedLoopEigVals)
    
    def StateFeedBack(self):
        self.feedBack = self.x-(matrix.A-matrix.B*self.K)*self.x #+ self.K*Xd
        #print("----------------------")
        print(self.feedBack) 
        print("----------------------")
        
    def SetCommand(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        self.command.linear.x = linear_x
        self.command.linear.y = linear_y
        self.command.linear.z = linear_z
        self.command.angular.x = angular_x
        self.command.angular.y = angular_y
        self.command.angular.z = angular_z
        self.pubCommand.publish(self.command)
        
    def SendLand(self):
        self.pubLand.publish(Empty())
        
    def SendTakeOff(self):
        self.pubTakeoff.publish(Empty()) 
        self.rate.sleep()        
         
    def Update(self):
        #if rospy.is_shutdown():
        #   return
        if uav.Z <= 60:
            self.SendLand()    
        if int(self.feedBack[0]) != int(self.desire[0]):
            X = 0
        else:
            X = 0
        if int(self.feedBack[2]) != int(self.desire[2]):
            Y = 0
        else:
            Y = 0
            
        if int(self.feedBack[4]) > int(self.desire[4]):
            Z = -0.1#np.absolute(int(self.feedBack[4]))
            
        elif int(self.feedBack[4]) < int(self.desire[4]):
            Z = 0.1#-np.absolute(int(self.feedBack[4]))
        else:
            Z = 0
            
        #"""    
        #altitude
        if int(self.feedBack[6]) > int(self.desire[6]):
            if uav.altd > 600:
                alt = -2.0
            else:
                alt = -0.2
            
        elif int(self.feedBack[6]) < int(self.desire[6]):
            alt = 0.5           
        else:
            #"""
            alt=0
          
        self.SetCommand(0,0,alt,X,Y,Z)
        print(uav.Z)
        #print(uav.yaw)
        
            
if __name__=='__main__':
    try:        
        while not rospy.is_shutdown():
            lqr = Lqr()
            lqr.SetGain()
            lqr.StateFeedBack()
            lqr.Update() 
            print("----------------------")
            lqr.rate.sleep()
    except rospy.ROSInterruptException:
        pass