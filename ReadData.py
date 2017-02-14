#!/usr/bin/env python 

import rospy

from std_msgs.msg import String 
from std_msgs.msg import Empty 
from ardrone_autonomy.msg import Navdata

class ReadData():
    def __init__(self):
        rospy.init_node('ReadData', anonymous=False)
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
        
        self.rotorA = 0 #rad/m
        self.rotorB = 0 #rad/m 
        self.rotorC = 0 #rad/m
        self.rotorD = 0 #rad/m
        self.rotorR = 0
        
        
        
        
    def ReceiveNavData(self,navdata):
        self.roll = navdata.rotX
        self.pitch = navdata.rotY
        self.yaw = navdata.rotZ
        self.X = navdata.ax
        self.Y = navdata.ay
        self.Z = navdata.az
        self.altd = navdata.altd
        self.rotorA = navdata.motor1
        self.rotorB = navdata.motor2
        self.rotorC = navdata.motor3
        self.rotorD = navdata.motor4
        self.rotorR = 0
        
        self.altd = navdata.altd
        
    
if __name__ == '__main__':
    try:
        uav = ReadData()
        while not rospy.is_shutdown():
            print("\n\n\n")
            print("Rol\t= "+ str(uav.roll))
            print("Pitch\t= "+ str(uav.pitch))
            print("Yaw\t= "+ str(uav.yaw))
            print("X\t= "+ str(uav.X))
            print("Y\t= "+ str(uav.Y))
            print("Z\t= "+ str(uav.Z))
            print("Motor 1\t= "+ str(uav.rotorA))
            print("Motor 2\t= "+ str(uav.rotorB))
            print("Motor 3\t= "+ str(uav.rotorC))
            print("Motor 4\t= "+ str(uav.rotorD))
    except rospy.ROSInterruptException:
        pass
        
    
