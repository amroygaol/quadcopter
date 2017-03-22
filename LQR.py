#!/usr/bin/env python
"""
Created on Thu Feb 16 13:27:16 2017

@author: SKRIPSI_LQR
"""

from __future__ import print_function, division

import controlpy
import control
import numpy as np
from ReadData import uav
from plant import matrix

class Lqr():
    def __init__(self):
        self.K = []
        self.sys = []
        self.x = np.matrix([[uav.roll],
                            [uav.roll_dot],
                            [uav.pitch],
                            [uav.pitch_dot],
                            [uav.yaw],
                            [uav.yaw_dot],
                            [uav.Z],
                            [uav.Z_dot]])
 
    def setGain(self):
        matrix.VarMatrix()
        matrix.SetMatrix()
        #menghitung gain K, persamaan riccati X, dan eigen value matriks
        self.K, X, closedLoopEigVals = controlpy.synthesis.controller_lqr(matrix.A, matrix.B, matrix.Q, matrix.R)
        #print(closedLoopEigVals)
        #menghitung Xdot dan y

    def StateSpace(self):
        matrix.VarMatrix()
        matrix.SetMatrix()
        #x_dot = (matrix.A - (matrix.B*self.K))*self.x
        self.sys = control.ss(matrix.A, matrix.B,matrix.C, matrix.D)
        
    def StateFeedBack(self):
        feedBack = (matrix.A-matrix.B*self.K)*self.x     
        print(feedBack) 
        print("----------------------")
        
    def SetCommand(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        self.command.linear.x = linear_x
        self.command.linear.y = linear_y
        self.command.linear.z = linear_z
        self.command.angular.x = angular_x
        self.command.angular.y = angular_y
        self.command.angular.z = angular_z
        self.pubCommand.publish(self.command)
        self.rate.sleep()
        
if __name__=='__main__':
    try:        
        for i in range(0,1):
            lqr = Lqr()
            #print(matrix.A)
            lqr.setGain()
            lqr.StateSpace()
            #print(a.K)
            lqr.StateFeedBack()
        
    except:
        pass