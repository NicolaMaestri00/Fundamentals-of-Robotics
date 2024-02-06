# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os
import math

dt = 0.001                   # controller time step
exp_duration = 7.0           # simulation duration
SLOW_FACTOR = 10             # to slow down simulation
frame_name = 'ee_link'       # name of the frame to control (end-effector) in the URDF

# Initial Conditions
q0 =   np.array([0.0, 0.0, 0.0, 0.0, 0.0])			# position
qd0 =  np.array([0.0, 0.0, 0.0, 0.0, 0.0])                    # velocity
qdd0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])                    # accelerations

# q_des
q_d = np.array([-4.24874,  2.26556, -0.21445, -0.41145, -4.4721 ])

# Postural task
Kp_postural = 350.0
Kd_postural = 15.0

null_space_task = np.array([0.52333, 0.0, 0.5, 1.57, 1.57])

## Matrix of gains
# P linear gain
Kx= np.eye(3)
Kx[0,0] = 1000
Kx[1,1] = 1000
Kx[2,2] = 1000
# P angular gain
Dx = np.eye(3)
Dx[0,0] = 500
Dx[1,1] = 500
Dx[2,2] = 500

# P angular gain
Ko= np.eye(3)
Ko[0,0] = 800
Ko[1,1] = 800
Ko[2,2] = 800
# D angular gain
Do= np.eye(3)
Do[0,0] = 30
Do[1,1] = 30
Do[2,2] = 30



## Parameters of the reference Cartesian sinusoidal trajectory (1, 2, 3, 4, 5, 6)
exp_duration_sin = exp_duration - 1.0               # sine reference duration
amp= np.array([ 0.1, 0.0, 0.0])                     # amplitude
phi =np.array([ 0.0, 0.0, 0.0])                     # phase
freq=np.array([ 1.5, 0.0, 0.0])                     # frequency


# EXE 2-3: Add external force
# Value of linear external force
extForce = np.array([0.0, 0.0, 200.0])
EXTERNAL_FORCE = False

RemoveInitialError = False
