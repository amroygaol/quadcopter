# -*- coding: utf-8 -*-
"""
Created on Tue Feb 14 09:53:58 2017

@author: SKRIPSI_LQR
"""

import rospy

from std_msgs.msg import String
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata

class ReadData():
    def __init__(self):
        self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavData)
        self.roll = 0
        self.roll_dot = 0
        self.pitch = 0
        self.pitch_dot = 0
        self.yaw = 0
        self.yaw_dot = 0
        self.X = 0
        self.X_dot = 0
        self.Y = 0
        self.Y_dot = 0
        self.Z = 0
        self.Z_dot = 0
        self.altd = 0
        
        
    def ReceiveNavData(self,navdata):
        self.roll = navdata.rotX
        self.pitch = navdata.rotY
        self.yaw = navdata.rotZ
        self.X = navdata.x
        self.Y = navdata.y
        self.Z = navdata.z
        self.altd = navdata.altd
    
        
    
if __name__ == '__main__':
    try:
        uav = ReadData()
        print("Rol\t= "+ str(uav.roll))
        print("Pitch\t= "+ str(uav.pitch))
        print("Yaw\t= "+ str(uav.yaw))
        print("X\t= "+ str(uav.X))
        print("Y\t= "+ str(uav.Y))
        print("Z\t= "+ str(uav.Z))
    except rospy.ROSInterruptException:
        pass
        
    