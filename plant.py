#!/usr/bin/env python 
"""
Created on Thu Feb  9 10:52:18 2017

@author: SKRIPSI_LQR
"""

#import rospy
import numpy as np
from ReadData import uav

class Plant():
    def __init__(self):
        
        self.l = 0.1785                     #mjarak antara rotor dan pusat massa
        self.Jr = 2.20321*10**-5            #kgm^2
        self.b = 1.27*10**-7                #g/rpm^2 thrust coefficient
        self.d = 3.19*10**-11               #g/rpm^2 drag  coefficient    
        self.g = 9.80665                    #m/s**2 gravity
        self.m = 0.38                       # or 0.428  kg
        
        #variabel momen inersia dari quadrotor
        self.Ixx = 2.2383*10**-3            #kgm^2
        self.Iyy = 2.9858*10**-3            #kgm^2
        self.Izz = 4.8334*10**-3            #kgm^2

        #deklarasi nilai konstanta a1..a5 dan b1..b3
        self.a1 = 0.8254478845552427
        #(self.Iyy - self.Izz) / self.Ixx
        self.a2 = -0.00984322923647411
        #(-self.Jr / self.Ixx)
        self.a3 = 0.15465303926842378
        #(self.Izz - self.Ixx) / self.Iyy
        self.a4 = 0.004558302644101462
        #(self.Jr/self.Y_inertia)
        self.a5 = -0.8691472972067786
        #(self.Ixx -self. Iyy) / self.Izz
        self.b1 = 79.74802305321
        #1 /self.Ixx
        self.b2 = 36.93052509620557
        #1 /self.Iyy
        self.b3 = 59.78297273762476
        #1 /self.Izz
        
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
        self.A = np.matrix([])
        self.B = np.matrix([])
        self.C = np.matrix([])
        self.D = np.matrix([])
        self.Q = np.matrix([])
        self.R = np.matrix([])
        
    #method untuk menghitung variabel a24, a42, a64
    def VarMatrix(self):
        self.a24 = uav.yaw*self.a1 + self.a2*uav.rotorR
        self.a42 = uav.yaw*self.a3 + self.a4*uav.rotorR
        self.a64 = uav.roll_dot*self.a5
        
    def SetMatrix(self):
        self.A = np.matrix([[0, 1, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, self.a24, 0, 0, 0, 0],
                                   [0, 0, 0, 1, 0, 0, 0, 0],
                                   [0, self.a42, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 1, 0, 0],
                                   [0, 0, 0, self.a64, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 1],
                                   [0, 0, 0, 0, 0, 0, 0, 0]])
                                   
    
    #imput matriks
        self.B = np.matrix([[0, 0, 0, 0],
                                   [0, self.b1,0, 0],
                                   [0, 0, 0, 0],
                                   [0, 0,self.b2, 0],
                                   [0, 0, 0, 0],
                                   [0, 0, 0,self.b3],
                                   [0, 0, 0, 0],
                                   [0, 0, 0, 1]])
    
   #output matriks 
                                   
        self.C = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 1, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 1, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 1, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 1, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 1, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 1, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 1]])
                                   
        self.D = np.matrix([[0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0]])
    
        self.Q = np.matrix([[0.0005, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0.0005, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0.0005, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0.0005, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0.0005, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0.0005, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0.0005, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0.0005]])
        
        self.R = np.matrix([[0.1, 0, 0, 0],
                                   [0, 0.1, 0, 0],
                                   [0, 0, 0.1, 0],
                                   [0, 0, 0, 0.1]])
                

    def SetInputSignal(self):
        self.U1 = self.b(uav.rotorA**2 + uav.rotorB**2 - uav.rotorC**2 + uav.rotorD**2)
        self.U2 = self.b(-uav.rotorB**2 + uav.rotorD**2)
        self.U3 = self.b(uav.rotorA**2 - uav.rotorC**2)
        self.U4 = self.d(-uav.rotorA**2 + uav.rotorB**2 - uav.rotorC**2 + uav.rotorD**2)
        

#instance data dari plant
matrix = Plant()
