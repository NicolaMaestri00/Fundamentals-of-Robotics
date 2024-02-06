# things to import
from __future__ import print_function
import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm

from base_controllers.utils.common_functions import *
from base_controllers.utils.ros_publish import RosPub
from base_controllers.utils.microfono_kin_dyn_utils import directKinematics
from base_controllers.utils.microfono_kin_dyn_utils import computeEndEffectorJacobian
from base_controllers.utils.microfono_kin_dyn_utils import numericalInverseKinematics as ik
from base_controllers.utils.microfono_kin_dyn_utils import fifthOrderPolynomialTrajectory as coeffTraj
from base_controllers.utils.microfono_kin_dyn_utils import geometric2analyticJacobian
from base_controllers.utils.math_tools import Math
import matplotlib.pyplot as plt
from base_controllers.utils.common_functions import plotJoint

import microfono_conf as conf

os.system("killall rosmaster rviz")
#instantiate graphic utils
ros_pub = RosPub("microfono")
robot = getRobotModel("microfono")

# Init variables
zero = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
time = 0.0
math_utils = Math()

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_id = robot.model.getFrameId(conf.frame_name)

################################################
# DIRECT KINEMATICS
################################################
# direct kinematics function
q = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
qd = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  
T_w0, T_w1, T_w2, T_w3, T_w4, T_w5, T_we = directKinematics(q)

# compare with Pinocchio built-in functions
robot.computeAllTerms(q, qd)
x = robot.framePlacement(q, frame_id).translation
o = robot.framePlacement(q, frame_id).rotation
position_diff = x - T_we[:3,3]
rotation_diff = o - T_we[:3,:3]

print('\n------------------------------------------------------------------------------------')
print('DIRECT KINEMATICS')
print()
print("Direct Kinematics - ee position, differece with Pinocchio library:", position_diff)
print("Direct Kinematics - ee orientation, differece with Pinocchio library:\n", rotation_diff)
print()
print('Matrice T_we')
print(T_we)
print()
print('Posizione con Pinocchio')
print(x)
print('Matrice wRe con Pinocchio')
print(o)

################################################
# GEOMETRIC JACOBIAN
################################################
J, z1, z2, z3, z4, z5 = computeEndEffectorJacobian(q)
# compare with Pinocchio
Jee = robot.frameJacobian(q, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
jacobian_diff = J - Jee

print()
print('GEOMETRIC JACOBIAN')
print(J)
print()
print('GEOMETRIC JACOBIAN con Pinocchio')
print(Jee)
print()
print("Direct Kinematics - ee Gometric Jacobian (6X4 matrix), differece with Pinocchio library:\n", jacobian_diff)

################################################
# ANALYTIC JACOBIAN
################################################
J_a = geometric2analyticJacobian(J, T_we)
print()
print("ANALYTIC JACOBIAN:\n", J_a)

################################################
# INVERSE KINEMATICS
################################################

# desired task space position
p = np.array([1, 2, 1, math.pi/3])

# initial guess (elbow up)
q_i  = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0])

# solution of the numerical ik
q_f, log_err, log_grad = ik(p, q_i, line_search = False, wrap = False)

# sanity check
# compare solution with values obtained through direct kinematics
T_w0, T_w1, T_w2, T_w3, T_w4, T_w5, T_we = directKinematics(q_f)
rpy = math_utils.rot2eul(T_we[:3,:3])
task_diff = p - np.hstack((T_we[:3,3],rpy[0]))

print()
print("Desired End effector \n", p)
print("Point obtained with IK solution \n", np.hstack((T_we[:3, 3], rpy[0])))
print("Norm of error at the end-effector position: \n", np.linalg.norm(task_diff))
print("Final joint positions\n", q_f)



################################################
# POLYNOMIAL TRAJECTORY IN THE JOINT SPACE
################################################
"""
# desired task space position
p = np.array([-2, 3, 1, math.pi/3])

# initial guess (elbow up)
q_i  = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0])

# solution of the numerical ik
q_f, log_err, log_grad = ik(p, q_i, line_search = False, wrap = False)
# Init loggers
q_log = np.empty((5))*nan
qd_log = np.empty((5))*nan
qdd_log = np.empty((5))*nan
time_log =  0

q = conf.q0
qd = conf.qd0
qdd = conf.qdd0


tm.sleep(1.)
ros_pub.publish(robot, conf.q0)
tm.sleep(2.)
while np.count_nonzero(q - q_f) :
    # Polynomial trajectory
    for i in range(5):
        a = coeffTraj(3.0, conf.q0[i], q_f[i])
        q[i] = a[0] + a[1]*time + a[2]*time**2 + a[3]*time**3 + a[4]*time**4 + a[5]*time**5
        qd[i] = a[1] + 2 * a[2] * time + 3 * a[3] * time ** 2 + 4 * a[4] * time ** 3 + 5 * a[5] * time ** 4
        qdd[i] = 2 * a[2] + 6 * a[3] * time + 12 * a[4] * time ** 2 + 20 * a[5] * time ** 3

    # update time
    time = time + conf.dt

    # Log Data into a vector
    time_log = np.append(time_log, time)
    q_log = np.vstack((q_log, q ))
    qd_log= np.vstack((qd_log, qd))
    qdd_log= np.vstack((qdd_log, qdd))

    #publish joint variables
    ros_pub.publish(robot, q, qd)
    ros_pub.add_marker(p)
    ros.sleep(conf.dt*conf.SLOW_FACTOR)

    # stops the while loop if  you prematurely hit CTRL+C
    if ros_pub.isShuttingDown():
        print ("Shutting Down")
        break
#plot position
q_0 = q_log[:,0]
q_0[0] = 0.0
q_1 = q_log[:,1]
q_1[0] = 0.0
q_2 = q_log[:,2]
q_2[0] = 0.0
q_3 = q_log[:,3]
q_3[0] = 0.0
q_4 = q_log[:,4]
q_4[0] = 0.0
#print(q_log[:,0])
#plotEndeff('ee position', 1, time_log, p_0_log.T, p_des_log.T)
plt.plot(time_log, q_0, label='q_0')
plt.plot(time_log, q_1, label='q_1')
plt.plot(time_log, q_2, label='q_2')
plt.plot(time_log, q_3, label='q_3')
plt.plot(time_log, q_4, label='q_4')
plt.title('joints positions')
plt.legend()

plt.show(block=True)
"""

################################################
# POLYNOMIAL TRAJECTORY IN THE TASK SPACE
################################################
q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

T_w0, T_w1, T_w2, T_w3, T_w4, T_w5, T_we = directKinematics(q)
p_0 = T_we[:3,3]
R = T_we[:3, :3]
rpy = math_utils.rot2eul(R)
roll = rpy[0]
p_0 = np.append(p_0, roll)

p_des = [0, 3, 1, math.pi/6]

print('Actual position:', p_0)
print('Desired position:', p_des)

# Init loggers
qd_log = np.empty((5))*nan
qdd_log = np.empty((5))*nan

p_0_log = np.empty((4))*nan
p_des_log = np.empty((4))*nan
q_log = np.empty((5))*nan
time_log =  0


tm.sleep(1.)
ros_pub.publish(robot, conf.q0)
tm.sleep(2.)


while np.count_nonzero(p_0 - p_des):

    # Polynomial trajectory in the task space
    for i in range(3):
        p_0[i] = p_0[i] + (p_des[i] - p_0[i]) * (time + conf.dt)

    # update time
    time = time + conf.dt

    # configuration of the joints
    q_f, log_err, log_grad = ik(p_0, q, line_search=False, wrap=False)

    # Log Data into a vector
    time_log = np.append(time_log, time)
    p_0_log = np.vstack((p_0_log, p_0 ))
    p_des_log = np.vstack((p_des_log, p_des))
    q_log = np.vstack((q_log, q_f ))

    # update joint state
    q = q_f

    #publish joint variables
    ros_pub.publish(robot, q, qd)
    ros_pub.add_marker(p_des)
    ros.sleep(conf.dt*conf.SLOW_FACTOR)

    # stops the while loop if  you prematurely hit CTRL+C
    if ros_pub.isShuttingDown():
        print ("Shutting Down")
        break


#plot position
q_0 = q_log[:,0]
q_0[0] = 0.0
q_1 = q_log[:,1]
q_1[0] = 0.0
q_2 = q_log[:,2]
q_2[0] = 0.0
q_3 = q_log[:,3]
q_3[0] = 0.0
q_4 = q_log[:,4]
q_4[0] = 0.0

#plotEndeff('ee position', 1, time_log, p_0_log.T, p_des_log.T)
plt.plot(time_log, q_0, label='q_0')
plt.plot(time_log, q_1, label='q_1')
plt.plot(time_log, q_2, label='q_2')
plt.plot(time_log, q_3, label='q_3')
plt.plot(time_log, q_4, label='q_4')
plt.title('joints positions')
plt.legend()

plt.show(block=True)








