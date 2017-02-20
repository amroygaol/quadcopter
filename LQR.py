#!/usr/bin/env python 
"""
Created on Thu Feb 16 13:27:16 2017

@author: SKRIPSI_LQR
"""

import numpy as np
import math
import controlpy
from scipy import signal

from plant import *
from ReadData import *

from matplotlib.pyplot import *
from control.matlab import *

x = np.matrix([[navData.roll],
              [navData.roll_dot],
              [navData.pitch],
              [navData.pitch_dot],
              [navData.yaw],
              [navData.yaw_dot],
              [navData.X],
              [navData.X_dot],
              [navData.Y],
              [navData.Y_dot],
              [navData.Z],
              [navData.Z_dot]])


#menghitung gain K, persamaan riccati X, dan eigen value matriks
K, X, closedLoopEigVals = controlpy.synthesis.controller_lqr(LqrVar.A_matrix, LqrVar.B_matrix, LqrVar.Q_matrix, LqrVar.R_matrix)


u = -Kx

#menghitung Xdot dan y

Xdot = (LqrVar.A_matrix*x) + (LqrVar.B_matrix*u)
y = (LqrVar.C_matrix*x) + (LqrVar.D_matrix*u)




