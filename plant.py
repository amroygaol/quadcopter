#!/usr/bin/env python 
"""
Created on Thu Feb  9 10:52:18 2017

@author: SKRIPSI_LQR
"""
import numpy as np
import math

import ReadData

#import rospy

from ardrone_autonomy.msg import Navdata
from matplotlib.pyplot import *
from control.matlab import *

from ReadData import *


class plant():
    #inialisasi atribut LQR
    def __init__(self):
        
        self.l = 0.17 #mjarak antara rotor dan pusat massa
        self.Jr = 0 #?
        self.b = 0 #..?
        self.d = 0        
        self.m = 0.38
        self.ux = 0
        self.uy = 0
        
        #variabel momen inersia dari quadrotor
        self.X_inertia  = 0
        self.Y_inertia = 0
        self.Z_inertia = 0

        #deklarasi nilai konstanta a1..a5 dan b1..b3
        self.a1 = (Y_inertiaf - Z_inertia) / X_inertia
        self.a2 = (-Jr / X_inertia)
        self.a3 = (Z_inertia - X_inertia) / Y_inertia
        self.a4 = (Jr/Y_inertia)
        self.a5 = (X_inertia - Y_inertia) / Z_inertia
        self.b1 = l/X_inertia
        self.b2 = l/Y_inertia
        self.b3 = 1/Z_inertia
        
        #deklarasi variabel input U1..U4
        self.U1 = 0
        self.U2 = 0
        self.U3 = 0
        self.U4 = 0
        

        #variabel untuk matriks A
        self.a24 = 0
        self.a42 = 0
        self.a64 = 0
        self.b81 = 0
        self.b101 = 0
        self.b121 = 0
        
        self.A_matrix = np.matrix([ [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, a24, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, a42, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, a64, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
                                   
    
    #imput matriks
        self.B_matrix = np.matrix([[0, 0, 0, 0],
                                   [0, b1,0, 0],
                                   [0, 0, 0, 0],
                                   [0, 0,b2, 0],
                                   [0, 0, 0, 0],
                                   [0, 0, 0,b3],
                                   [0, 0, 0, 0],
                                   [b81, 0, 0, 0],
                                   [0, 0, 0, 0],
                                   [b101,0, 0, 0],
                                   [0, 0, 0, 0],
                                   [b121,0, 0, 0]])
    
   #output matriks 
                                   
        self.C_matrix = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
                                   
        self.D_matrix = np.matrix([[0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0]])
        self.Q_matrix = np.matrix([])
        self.R_matrix = np.matrix([])
        
        self.altd2 = 2000
        self.altd = 0
        

    def SetInputSignal(self):
        self.U1 = self.b(navData.rotorA**2 + navData.rotorB**2 - navData.rotorC**2 + navData.rotorD**2)
        self.U2 = self.b(-navData.rotorB**2 + navData.rotorD**2)
        self.U3 = self.b(navData.rotorA**2 - navData.rotorC**2)
        self.U4 = self.d(-navData.rotorA**2 + navData.rotorB**2 - navData.rotorC**2 + navData.rotorD**2)
        
        #SET nilai ux dan uy
    def SetUxUy(self):
        self.ux = (math.cos(navData.roll)*math.sin(navData.pitch)*math.cos(navData.yaw) + math.sin(navData.roll)*math.sin(navData.yaw))
        self.uy = (math.cos(navData.roll)*math.sin(navData.pitch)*math.sin(navData.yaw) - math.sin(navData.roll)*math.cos(navData.yaw))
        
    #method untuk menghitung variabel a24, a42, a64
    def VarMatriks(self):
        self.a24 = navData.yaw*self.a1 + self.a2*navData.rotorR
        self.a42 = navData.yaw*self.a3 + self.a4*navData.rotorR
        self.a64 = navData.roll_dot*self.a5
        self.b81 = (math.cos(navData.roll)*math.cos(navData.pitch))/self.m
        self.b101 = self.ux / self.m
        self.b121 = self.uy / self.m
    #method untuk menghitung 
        
        
   # def GetGain_X_EigenVal(self):
