#!/usr/bin/env python 
"""
Created on Thu Feb 16 13:27:16 2017

@author: AMROY_GAOL
"""

import rospy
from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import Imu

class ReadData():
    def __init__(self):
        #rospy.init_node('ReadData', anonymous=False)
        self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavData)
        self.imuData = rospy.Subscriber('ardrone/imu',Imu, self.GetImuData)
        self.roll = 0
        self.roll_dot = 0
        self.pitch = 0
        self.pitch_dot = 0
        self.yaw = 0
        self.yaw_dot = 0
        self.Z = 0
        self.Z_dot = 0
    
        self.rotorR = 2000
                
        
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
        self.Z = navdata.altd
        self.Z_dot = navdata.vz
        
    def GetImuData(self,Imu):
        self.roll_dot = Imu.angular_velocity.x
        self.pitch_dot = Imu.angular_velocity.y
        self.yaw_dot = Imu.angular_velocity.z
        
     
uav = ReadData()
        
"""   
if __name__ == '__main__':
    try:
        uav = ReadData()
        while not rospy.is_shutdown():
            print("-------------------")            
            print("Rol\t\t= "+ str(uav.roll))
            print("Rol_dot\t\t= "+ str(uav.roll_dot))            
            print("Pitch\t\t= "+ str(uav.pitch))
            print("PitchDot\t= "+ str(uav.pitch_dot))
            print("Yaw\t\t= "+ str(uav.yaw))
            print("Yaw_dot\t\t= "+ str(uav.yaw_dot))
            print("Z\t\t= "+ str(uav.Z))
            print("Z_dot\t\t= "+ str(uav.Z_dot))
            print("-------------------")
    except rospy.ROSInterruptException:
        pass
"""    

