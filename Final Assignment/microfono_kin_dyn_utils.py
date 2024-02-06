# -*- coding: utf-8 -*-
"""
Created on May 4 2021

@author: ovillarreal
"""
from __future__ import print_function
import numpy as np
import os
import math
import pinocchio as pin
from pinocchio.utils import *
from base_controllers.utils.math_tools import Math
import time as tm


def setRobotParameters():
    # link lengths
    l1 = 4  # relative displacement of link_01 along base_link ( Z-axis )
    l2 = 1  # relative displacement of link_03 along link_02 ( X-axis )
    l3 = 2  # relative displacement of link_04 along link_03 ( X-axis )
    l4 = 1  # relative displacement of link_05 along link_04 ( X-axis )
    l5 = 0.2  # relative displacement of ee_link along link_05 ( X-axis )

    lengths = np.array([l1, l2, l3, l4, l5])

    m1 = 1 # base_link
    m2 = 1 # link_01
    m3 = 1 # link_02
    m4 = 1 # link_03
    m5 = 1 # link_04
    m6 = 1 # link_05

    link_masses = np.array([m1, m2, m3, m4, m5, m6])

    # com of the links expressed in the respective link frame
    # com_link =  model.inertias[idx].lever (Pinocchio)
    com0 = np.array([ 0., 0., 0.])  # base link
    com1 = np.array([ 0., 0., 0.])  # link_01
    com2 = np.array([1.5, 0., 0.])  # link_02
    com3 = np.array([1.0, 0., 0.])  # link_03
    com4 = np.array([0.5, 0., 0.])  # link_04
    com0 = np.array([0.1, 0., 0.])  # link_05
    # w_com_link = data.oMi[idx].rotation.dot(com_link) + data.oMi[idx].translation

    # inertia tensors of the links  w.r.t. to own CoM of each link expressed in the respective link frames
    I_0 = np.array([[(1/12)*(1+0.05**2),                0.0,          0.0],
                    [               0.0, (1/12)*(1+0.05**2),          0.0],
                    [               0.0,                0.0, (1/12)*(1+1)]])
                    
    I_1 = np.array([[(1/2)*(0.05**2),             0.0,            0.0],
                    [            0.0, (1/2)*(0.05**2),            0.0],
                    [            0.0,             0.0, (1/2)*(0.4**2)]])
                    
    I_2 = np.array([[   (1/2)*(3**2),             0.0,             0.0],
                    [            0.0,    (1/2)*(3**2),             0.0],
                    [            0.0,             0.0, (1/2)*(0.05**2)]])               

    I_3 = np.array([[   (1/2)*(2**2),             0.0,             0.0],
                    [            0.0,    (1/2)*(2**2),             0.0],
                    [            0.0,             0.0, (1/2)*(0.05**2)]])  
                                 
    I_4 = np.array([[   (1/2)*(1**2),             0.0,             0.0],
                    [            0.0,    (1/2)*(1**2),             0.0],
                    [            0.0,             0.0, (1/2)*(0.02**2)]])   
                                
    I_5 = np.array([[ (1/2)*(0.2**2),             0.0,             0.0],
                    [            0.0,  (1/2)*(0.2**2),             0.0],
                    [            0.0,             0.0, (1/2)*(0.03**2)]])               
                       

    inertia_tensors = np.array([I_0, I_1, I_2, I_3, I_4, I_5])


    return lengths, inertia_tensors, link_masses, coms


def directKinematics(q):
    
    # link lengths
    l1 = 4  # relative displacement of base_link w.r.t. world_frame ( Z-axis )
    l2 = 1  # relative displacement of link_03 w.r.t. link_02 ( X-axis )
    l3 = 2  # relative displacement of link_04 w.r.t. link_03 ( X-axis )
    l4 = 1  # relative displacement of link_05 w.r.t. link_04 ( X-axis )
    l5 = 0.2  # relative displacement of ee_link w.r.t. link_05 ( X-axis )

    q1 = q[0]  # joint_1
    q2 = q[1]  # joint_2
    q3 = q[2]  # joint_3
    q4 = q[3]  # joint_4
    q5 = q[4]  # joint_5

    # LOCAL homogeneous transformation matrices (base link is 0)
 
    # world_frame__base_link
    # rigid transform (translation l1 along Z axis, no rotation)   
    T_w0 = np.array([[ 1,  0,  0,   0],
                     [ 0,  1,  0,   0],
                     [ 0,  0,  1,  l1],
                     [ 0,  0,  0,   1]])

    # base_link__link_01
    # rigid transform (no translation, pure rotation)
    T_01r = np.array([[ 1,  0,  0,   0],
                      [ 0, -1,  0,   0],
                      [ 0,  0, -1,   0],
                      [ 0,  0,  0,   1]])
    # joint transform (rotation about Z axis)
    T_1r1 = np.array([[math.cos(q1), -math.sin(q1),       0      ,       0      ],
                      [math.sin(q1),  math.cos(q1),       0      ,       0      ],
                      [     0      ,       0      ,       1      ,       0      ],
                      [     0      ,       0      ,       0      ,       1      ]])
    # local hom. transform from base_link to link_01
    T_01 = T_01r.dot(T_1r1)

    # link_01__link_02
    # rigid transform (no translation, pure rotation)
    T_12r = np.array([[ 1,  0,  0,   0],
                      [ 0,  0, -1,   0],
                      [ 0,  1,  0,   0],
                      [ 0,  0,  0,   1]])
    # joint transform (rotation about Z axis)
    T_2r2 = np.array([[math.cos(q2), -math.sin(q2),       0      ,       0      ],
                      [math.sin(q2),  math.cos(q2),       0      ,       0      ],
                      [     0      ,       0      ,       1      ,       0      ],
                      [     0      ,       0      ,       0      ,       1      ]])
    # local hom. transform from link_01 to link_02
    T_12 = T_12r.dot(T_2r2)

    # link_02__link_03
    # rigid transform (translation l2 along X axis, no rotation)
    T_23r = np.array([[ 1,  0,  0,   l2],
                      [ 0,  1,  0,   0],
                      [ 0,  0,  1,   0],
                      [ 0,  0,  0,   1]])
    # joint transform (traslation along X axis)
    T_3r3 = np.array([[     1      ,       0      ,       0      ,      q3      ],
                      [     0      ,       1      ,       0      ,       0      ],
                      [     0      ,       0      ,       1      ,       0      ],
                      [     0      ,       0      ,       0      ,       1      ]])
    # local hom. transform from link_02 to link_03
    T_23 = T_23r.dot(T_3r3)

    # link_03__link_04
    # rigid transform (translation l3 along X axis, no rotation)
    T_34r = np.array([[ 1,  0,  0,   l3],
                      [ 0,  1,  0,   0],
                      [ 0,  0,  1,   0],
                      [ 0,  0,  0,   1]])
    # joint transform (rotation about Z axis)
    T_4r4 = np.array([[math.cos(q4), -math.sin(q4),       0      ,       0      ],
                      [math.sin(q4),  math.cos(q4),       0      ,       0      ],
                      [     0      ,       0      ,       1      ,       0      ],
                      [     0      ,       0      ,       0      ,       1      ]])
    # local hom. transform from link_03 to link_04
    T_34 = T_34r.dot(T_4r4)

    # link_03__link_04
    # rigid transform (translation l4 along X axis, no rotation)
    T_45r = np.array([[ 1,  0,  0,   l4],
                      [ 0,  1,  0,   0],
                      [ 0,  0,  1,   0],
                      [ 0,  0,  0,   1]])
    # joint transform (rotation about Z axis)
    T_5r5 = np.array([[math.cos(q5), -math.sin(q5),       0      ,       0      ],
                      [math.sin(q5),  math.cos(q5),       0      ,       0      ],
                      [     0      ,       0      ,       1      ,       0      ],
                      [     0      ,       0      ,       0      ,       1      ]])
    # local hom. transform from link_04 to link_05
    T_45 = T_45r.dot(T_5r5)

    # link_05__end-effector
    # rigid transform (translation l5 along X axis, no rotation)
    T_5e = np.array([[ 0,  0,  1,  l5],
                     [ 0,  1,  0,   0],
                     [-1,  0,  0,   0],
                     [ 0,  0,  0,   1]])

    # GLOBAL homogeneous transformation matrices
    T_w1 = T_w0.dot(T_01)
    T_w2 = T_w1.dot(T_12)
    T_w3 = T_w2.dot(T_23)
    T_w4 = T_w3.dot(T_34)
    T_w5 = T_w4.dot(T_45)
    T_we = T_w5.dot(T_5e)

    return T_w0, T_w1, T_w2, T_w3, T_w4, T_w5, T_we


'''
    This function computes the Geometric Jacobian of the end-effector expressed in the base link frame 
'''


def computeEndEffectorJacobian(q):
    # compute direct kinematics
    T_w0, T_w1, T_w2, T_w3, T_w4, T_w5, T_we = directKinematics(q)

    # link position vectors
    p_w1 = T_w1[:3, 3]
    p_w2 = T_w2[:3, 3]
    p_w3 = T_w3[:3, 3]
    p_w4 = T_w4[:3, 3]
    p_w5 = T_w5[:3, 3]
    p_we = T_we[:3, 3]

    # z vectors for rotations
    z1 = T_w1[:3, 2]  # Z axis
    z2 = T_w2[:3, 2]  # Z axis
    z3 = T_w3[:3, 0]  # X axis
    z4 = T_w4[:3, 2]  # Z axis
    z5 = T_w5[:3, 2]  # Z axis

    # vectors from link i to end-effector
    p_w_1e = p_we - p_w1
    p_w_2e = p_we - p_w2
    p_w_3e = p_we - p_w3
    p_w_4e = p_we - p_w4
    p_w_5e = p_we - p_w5


    # linear and angular part of Jacobian matrix
    J_p = np.hstack((np.cross(z1, p_w_1e).reshape(3, 1), np.cross(z2, p_w_2e).reshape(3, 1), z3.reshape(3, 1),
                     np.cross(z4, p_w_4e).reshape(3, 1), np.cross(z5, p_w_5e).reshape(3, 1)))
    J_o = np.hstack((z1.reshape(3, 1), z2.reshape(3, 1), np.zeros(3).reshape(3, 1), z4.reshape(3, 1), z5.reshape(3, 1)))

    # Jacobian matrix and joint axes both expressed in the world frame)
    J = np.vstack((J_p, J_o))

    return J, z1, z2, z3, z4, z5


def geometric2analyticJacobian(J, T_we):
    R_we = T_we[:3, :3]
    math_utils = Math()
    rpy_ee = math_utils.rot2eul(R_we)
    roll = rpy_ee[0]
    pitch = rpy_ee[1]
    yaw = rpy_ee[2]

    # compute the mapping between euler rates and angular velocity
    T_w = np.array([[math.cos(yaw) * math.cos(pitch), -math.sin(yaw), 0],
                    [math.sin(yaw) * math.cos(pitch), math.cos(yaw), 0],
                    [-math.sin(pitch), 0, 1]])

    T_a = np.array([np.vstack((np.hstack((np.identity(3), np.zeros((3, 3)))),
                               np.hstack((np.zeros((3, 3)), np.linalg.inv(T_w)))))])

    J_a = np.dot(T_a, J)

    return J_a[0]


def numericalInverseKinematics(p_d, q0, line_search=False, wrap=False):
    math_utils = Math()

    # hyper-parameters
    epsilon = 1e-06  # Tolerance for stropping criterion
    lambda_ = 1e-08  # Regularization or damping factor (1e-08->0.01)
    max_iter = 200  # Maximum number of iterations
    # For line search only
    # gamma = 0.5
    beta = 0.5  # Step size reduction

    # initialization of variables
    iter = 0
    alpha = 1  # Step size
    log_grad = []
    log_err = []
    
    #print('\nINIZIA LA INVERSE KINEMATICS\n')

    # Inverse kinematics with line search
    while True:
        # evaluate  the kinematics for q0
        J, _, _, _, _, _ = computeEndEffectorJacobian(q0)
        _, _, _, _, _, _, T_we = directKinematics(q0)

        p_e = T_we[:3, 3]
        R = T_we[:3, :3]
        rpy = math_utils.rot2eul(R)
        roll = rpy[0]
        p_e = np.append(p_e, roll)

        # error
        e_bar = p_e - p_d
        J_bar = geometric2analyticJacobian(J, T_we)
        # take first 4 rows correspondent to our task
        J_bar = J_bar[:4, :]
        # evaluate the gradient
        grad = J_bar.T.dot(e_bar)
        #print('ERRORE:\t',e_bar)

        log_grad.append(np.linalg.norm(grad))
        log_err.append(np.linalg.norm(e_bar))

        if np.linalg.norm(grad) < epsilon:
            #print("IK Convergence achieved!, norm(grad) :", np.linalg.norm(grad))
            #print("Inverse kinematics solved in {} iterations".format(iter))
            break
        if iter >= max_iter:
            #print(                "Warning: Max number of iterations reached, the iterative algorithm has not reached convergence to the desired precision. Error is:  ",                np.linalg.norm(e_bar))
            break
        # Compute the error
        JtJ = np.dot(J_bar.T, J_bar) + np.identity(J_bar.shape[1]) * lambda_
        JtJ_inv = np.linalg.inv(JtJ)
        P = JtJ_inv.dot(J_bar.T)
        dq = - P.dot(e_bar)

        if not line_search:
            q1 = q0 + dq * alpha
            q0 = q1
        else:
            #print("Iter # :", iter)
            # line search loop
            while True:
                # update
                q1 = q0 + dq * alpha
                # evaluate  the kinematics for q1
                _, _, _, _, _, T_0e1 = directKinematics(q1)
                p_e1 = T_0e1[:3, 3]
                R1 = T_0e1[:3, :3]
                rpy1 = math_utils.rot2eul(R1)
                roll1 = rpy1[0]
                p_e1 = np.append(p_e1, roll1)
                e_bar_new = p_e1 - p_d
                # print "e_bar1", np.linalg.norm(e_bar_new), "e_bar", np.linalg.norm(e_bar)

                error_reduction = np.linalg.norm(e_bar) - np.linalg.norm(e_bar_new)
                threshold = 0.0  # more restrictive gamma*alpha*np.linalg.norm(e_bar)

                if error_reduction < threshold:
                    alpha = beta * alpha
                    print(" line search: alpha: ", alpha)
                else:
                    q0 = q1
                    alpha = 1
                    break

        iter += 1

    # wrapping prevents from outputs outside the range -2pi, 2pi
    if wrap:
        for i in range(len(q0)):
            while q0[i] >= 2 * math.pi:
                q0[i] -= 2 * math.pi
            while q0[i] < -2 * math.pi:
                q0[i] += 2 * math.pi

    return q0, log_err, log_grad
    
    
def numericalInverseKinematics_1(p_d, q0, line_search=False, wrap=False):
    math_utils = Math()

    # hyper-parameters
    epsilon = 1e-06  # Tolerance for stropping criterion
    lambda_ = 1e-08  # Regularization or damping factor (1e-08->0.01)
    max_iter = 200  # Maximum number of iterations
    # For line search only
    # gamma = 0.5
    beta = 0.5  # Step size reduction

    # initialization of variables
    iter = 0
    alpha = 1  # Step size
    log_grad = []
    log_err = []

    # Inverse kinematics with line search
    while True:
        # evaluate  the kinematics for q0
        J, _, _, _, _, _ = computeEndEffectorJacobian(q0)
        _, _, _, _, _, _, T_we = directKinematics(q0)

        p_e = T_we[:3, 3]

        # error
        e_bar = p_e - p_d
        J_bar = geometric2analyticJacobian(J, T_we)
        # take first 3 rows correspondent to our task
        J_bar = J_bar[:3, :]
        # evaluate the gradient
        grad = J_bar.T.dot(e_bar)

        log_grad.append(np.linalg.norm(grad))
        log_err.append(np.linalg.norm(e_bar))

        if np.linalg.norm(grad) < epsilon:
            break
        if iter >= max_iter:
            break
        # Compute the error
        JtJ = np.dot(J_bar.T, J_bar) + np.identity(J_bar.shape[1]) * lambda_
        JtJ_inv = np.linalg.inv(JtJ)
        P = JtJ_inv.dot(J_bar.T)
        dq = - P.dot(e_bar)

        if not line_search:
            q1 = q0 + dq * alpha
            q0 = q1
        else:
            # line search loop
            while True:
                # update
                q1 = q0 + dq * alpha
                # evaluate  the kinematics for q1
                _, _, _, _, _, T_0e1 = directKinematics(q1)
                p_e1 = T_0e1[:3, 3]
                e_bar_new = p_e1 - p_d
                # print "e_bar1", np.linalg.norm(e_bar_new), "e_bar", np.linalg.norm(e_bar)

                error_reduction = np.linalg.norm(e_bar) - np.linalg.norm(e_bar_new)
                threshold = 0.0  # more restrictive gamma*alpha*np.linalg.norm(e_bar)

                if error_reduction < threshold:
                    alpha = beta * alpha
                else:
                    q0 = q1
                    alpha = 1
                    break

        iter += 1

    # wrapping prevents from outputs outside the range -2pi, 2pi
    if wrap:
        for i in range(len(q0)):
            while q0[i] >= 2 * math.pi:
                q0[i] -= 2 * math.pi
            while q0[i] < -2 * math.pi:
                q0[i] += 2 * math.pi

    return q0, log_err, log_grad


def fifthOrderPolynomialTrajectory(tf, start_pos, end_pos, start_vel=0, end_vel=0, start_acc=0, end_acc=0):
    # Matrix used to solve the linear system of equations for the polynomial trajectory
    polyMatrix = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0],
                           [0, 0, 2, 0, 0, 0],
                           [1, tf, np.power(tf, 2), np.power(tf, 3), np.power(tf, 4), np.power(tf, 5)],
                           [0, 1, 2 * tf, 3 * np.power(tf, 2), 4 * np.power(tf, 3), 5 * np.power(tf, 4)],
                           [0, 0, 2, 6 * tf, 12 * np.power(tf, 2), 20 * np.power(tf, 3)]])

    polyVector = np.array([start_pos, start_vel, start_acc, end_pos, end_vel, end_acc])
    matrix_inv = np.linalg.inv(polyMatrix)
    polyCoeff = matrix_inv.dot(polyVector)

    return polyCoeff


def RNEA(g0, q, qd, qdd, Fee=np.zeros(3), Mee=np.zeros(3)):
    # setting values of inertia tensors w.r.t. to their CoMs from urdf and link masses
    _, tensors, m, coms = setRobotParameters()

    # get inertia tensors about the CoM expressed in the respective link frame
    _0_I_0 = tensors[0]
    _1_I_1 = tensors[1]
    _2_I_2 = tensors[2]
    _3_I_3 = tensors[3]
    _4_I_4 = tensors[4]

    # get positions of the link CoM expressed in the respective link frame
    _0_com_0 = coms[0]
    _1_com_1 = coms[1]
    _2_com_2 = coms[2]
    _3_com_3 = coms[3]
    _4_com_4 = coms[4]

    # number of joints
    n = len(q)

    # pre-pend a fake joint for base link
    q_link = np.insert(q, 0, 0.0, axis=0)
    qd_link = np.insert(qd, 0, 0.0, axis=0)
    qdd_link = np.insert(qdd, 0, 0.0, axis=0)

    # initialation of variables
    zeroV = np.zeros(3)
    omega = np.array([zeroV, zeroV, zeroV, zeroV, zeroV])
    v = np.array([zeroV, zeroV, zeroV, zeroV, zeroV])
    omega_dot = np.array([zeroV, zeroV, zeroV, zeroV, zeroV])
    a = np.array([zeroV, zeroV, zeroV, zeroV, zeroV])
    vc = np.array([zeroV, zeroV, zeroV, zeroV, zeroV])
    ac = np.array([zeroV, zeroV, zeroV, zeroV, zeroV])

    # these arrays are 1 element longer than the others because in the back recursion we consider also the forces/moments coming from the ee
    F = np.array([zeroV, zeroV, zeroV, zeroV, zeroV, Fee])
    M = np.array([zeroV, zeroV, zeroV, zeroV, zeroV, Mee])

    tau = np.array([0.0, 0.0, 0.0, 0.0])

    # obtaining joint axes vectors required in the computation of the velocities and accelerations (expressed in the world frame)
    _, z1, z2, z3, z4 = computeEndEffectorJacobian(q)

    z = np.array([np.zeros(3), z1, z2, z3, z4])

    # global homogeneous transformation matrices
    T_01, T_02, T_03, T_04, T_0e = directKinematics(q)

    # link positions w.r.t. the world frame
    p_00 = np.array([0.0, 0.0, 0.0])
    p_01 = T_01[:3, 3]
    p_02 = T_02[:3, 3]
    p_03 = T_03[:3, 3]
    p_04 = T_04[:3, 3]
    p_0e = T_0e[:3, 3]

    # array used in the recursion (this array is 1 element longer than the others because in the back recursion we consider also the position of the ee)
    p = np.array([p_00, p_01, p_02, p_03, p_04, p_0e])

    # rotation matrices w.r.t. to the world of each link
    R_00 = np.eye(3)
    R_01 = T_01[:3, :3]
    R_02 = T_02[:3, :3]
    R_03 = T_03[:3, :3]
    R_04 = T_04[:3, :3]

    # positions of the CoMs w.r.t. to the world frame
    pc_0 = p_00 + _0_com_0
    pc_1 = p_01 + np.dot(R_01, _1_com_1)
    pc_2 = p_02 + np.dot(R_02, _2_com_2)
    pc_3 = p_03 + np.dot(R_03, _3_com_3)
    pc_4 = p_04 + np.dot(R_04, _4_com_4)

    # array used in the recursion
    pc = np.array([pc_0, pc_1, pc_2, pc_3, pc_4])

    # expressing tensors of inertia of the links (about the com) in the world frame (time consuming)
    I_0 = np.dot(np.dot(R_00, _0_I_0), R_00.T)
    I_1 = np.dot(np.dot(R_01, _1_I_1), R_01.T)
    I_2 = np.dot(np.dot(R_02, _2_I_2), R_02.T)
    I_3 = np.dot(np.dot(R_03, _3_I_3), R_03.T)
    I_4 = np.dot(np.dot(R_04, _4_I_4), R_04.T)

    # array used in the recursion
    I = np.array([I_0, I_1, I_2, I_3, I_4])

    # forward pass: compute accelerations from link 0 to  link 4, range(n+1) = (0, 1, 2, 3, 4)
    for i in range(n + 1):
        if i == 0:  # we start from base link 0
            p_ = p[0]
            # base frame is still (not true for a legged robot!)
            omega[0] = zeroV
            v[0] = zeroV
            omega_dot[0] = zeroV
            a[
                0] = -g0  # if we consider gravity as  acceleration (need to move to right hand side of the Newton equation) we can remove it from all the Netwon equations
        else:
            p_ = p[i] - p[i - 1]  # p_i-1,i
            omega[i] = omega[i - 1] + qd_link[i] * z[i]
            omega_dot[i] = omega_dot[i - 1] + qdd_link[i] * z[i] + qd_link[i] * np.cross(omega[i - 1], z[i])

            v[i] = v[i - 1] + np.cross(omega[i - 1], p_)
            a[i] = a[i - 1] + np.cross(omega_dot[i - 1], p_) + np.cross(omega[i - 1], np.cross(omega[i - 1], p_))

        pc_ = pc[i] - p[i]  # p_i,c

        # compute com quantities
        vc[i] = v[i] + np.cross(omega[i], p_)
        ac[i] = a[i] + np.cross(omega_dot[i], pc_) + np.cross(omega[i], np.cross(omega[i], pc_))

    # backward pass: compute forces and moments from wrist link (4) to base link (0)
    for i in range(n, -1, -1):
        # lever arms wrt to other link frames
        pc_ = p[i] - pc[i]
        pc_1 = p[i + 1] - pc[i]

        F[i] = F[i + 1] + m[i] * (ac[i])

        M[i] = M[i + 1] - \
               np.cross(pc_, F[i]) + \
               np.cross(pc_1, F[i + 1]) + \
               np.dot(I[i], omega_dot[i]) + \
               np.cross(omega[i], np.dot(I[i], omega[i]))

        # compute torque for all joints (revolute) by projection
    for joint_idx in range(n):
        tau[joint_idx] = np.dot(z[joint_idx + 1], M[joint_idx + 1])

    return tau


# computation of gravity terms
def getg(q, robot):
    qd = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    qdd = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    # Pinocchio
    g = pin.rnea(robot.model, robot.data, q,qd ,qdd)
    return g


# computation of generalized mass matrix
def getM(q, robot):
    n = len(q)
    M = np.zeros((n, n))
    for i in range(n):
        ei = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        ei[i] = 1
        # Pinocchio
        g = getg(q,robot)
        tau_p = pin.rnea(robot.model, robot.data, q, np.array([0,0,0,0,0]),ei) -g
        # fill in the column of the inertia matrix
        M[:5, i] = tau
    return M

# computation of C term
def getC(q, qd, robot):
    qdd = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    # Pinocchio
    g = getg(q,robot)
    C = pin.rnea(robot.model, robot.data,q,qd,qdd) - g
    return C





