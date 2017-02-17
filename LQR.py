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

from matplotlib.pyplot import *
from control.matlab import *

<<<<<<< HEAD


gain, X, closedLoopEigVals = controlpy.synthesis.controller_lqr(LqrVar.A_matrix, LqrVar.B_matrix, LqrVar.Q_matrix, LqrVar.R_matrix)

sys = signal.StateSpace(LqrVar.A_matrix, LqrVar.B_matrix, LqrVar.C_matrix, LqrVar.D_matrix)
t1, y1 = signal.step(sys)
