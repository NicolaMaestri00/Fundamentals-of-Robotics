# things to import
import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm

import os
from base_controllers.utils.common_functions import *
from base_controllers.utils.ros_publish import RosPub
from base_controllers.utils.microfono_kin_dyn_utils import RNEA
from base_controllers.utils.microfono_kin_dyn_utils import getM
from base_controllers.utils.microfono_kin_dyn_utils import getg
from base_controllers.utils.microfono_kin_dyn_utils import getC


import microfono_conf as conf

#instantiate graphic utils
os.system("killall rosmaster rviz")
ros_pub = RosPub("microfono")
robot = getRobotModel("microfono")

# Init variables
zero = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
time = 0.0

q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

q_des = zero
qd_des = zero
qdd_des = zero        # joint reference acceleration

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)

error = np.array([1, 1, 1, 1, 1])

########################
# SIMULATE DYNAMICS with Pinocchio
########################

# Main loop to simulate dynamics
while any(i >= 0.01 for i in np.abs(error)):

    # initialize Pinocchio variables
    robot.computeAllTerms(q, qd)
    # vector of gravity acceleration
    g0 = np.array([0.0, 0.0, -9.81])
    
    ##############################
    # RNEA with Pinocchio to compute tau
    ##############################

    tau = pin.rnea(robot.model, robot.data, q, qd, qdd)    

    ######################################
    # Compute dynamic terms with Pinocchio
    ######################################
    # gravity terms
    g = robot.gravity(q)

    # joint space inertia matrix
    M = robot.mass(q, False)
    
    # compute joint space intertia matrix with built-in pinocchio rnea
    # MEMO: M = Mp
    Mp  = np.zeros((5,5))
    for i in range(5):
      ei = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
      ei[i] = 1
      taup = pin.rnea(robot.model, robot.data, q, np.array([0,0,0,0,0]) ,ei)
      Mp[:5,i] = taup - g

    # Pinocchio bias terms
    hp = robot.nle(q, qd, False)
    c = getC(q,qd,robot)

    #############################################
    # Damping term
    #############################################
    # viscous friction to stop the motion
    # damping = zero
    damping =  - 30*qd

    #############################################
    # Joint accelerations
    #############################################
    # compute accelerations (without torques)
    qdd = np.linalg.inv(Mp).dot(damping-hp)

    #############################################
    # Simulate the dynamics
    #############################################
    # Forward Euler Integration
    qd = qd + qdd * conf.dt
    q = q + conf.dt * qd  + 0.5 * pow(conf.dt,2) * qdd
    
    #############################################
    # Vincular reaction for prismatic joint
    #############################################    
    
    if q[2]>=2.0:
       q[2] = 2.0
       qd[2] = 0.0
       qdd[2] = 0.0
       tau[2] = 0.0

    # update time
    time = time + conf.dt
                
    #publish joint variables
    ros_pub.publish(robot, q, qd)
    tm.sleep(conf.dt*conf.SLOW_FACTOR)
    
    print('\n-------------------------------------------------------------')
    print('Joint state')
    print('Positions:\t', q)
    print('Velocities:\t', qd)
    print('Accelerations:\t', qdd)
    print('Torques:\t', tau)
    
    # stops the while loop if  you prematurely hit CTRL+C                    
    if ros_pub.isShuttingDown():
        print ("Shutting Down")                    
        break
            
#raw_input("Robot came to a stop. Press Enter to continue")
ros_pub.deregister_node()






