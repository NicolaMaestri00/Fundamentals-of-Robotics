# things to import
from __future__ import print_function
import pinocchio as pin
from pinocchio.utils import *
from numpy import nan
import math
import time as tm
from base_controllers.utils.common_functions import *
from base_controllers.utils.ros_publish import RosPub
from base_controllers.utils.math_tools import Math
import microfono_conf as conf

#instantiate graphic utils
os.system("killall rosmaster rviz")
ros_pub = RosPub("microfono")
robot = getRobotModel("microfono")

math_utils = Math()
# Init variables
zero = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
zero_cart = np.array([ 0.0, 0.0, 0.0])
time = 0.0

two_pi_f             = 2*np.pi*conf.freq   # frequency (time 2 PI)
two_pi_f_amp         = np.multiply(two_pi_f, conf.amp) 
two_pi_f_squared_amp = np.multiply(two_pi_f, two_pi_f_amp)

# Init loggers
buffer_size = int(math.floor(conf.exp_duration/conf.dt))
log_counter = 0
p_log = np.empty((3, buffer_size))*nan
p_des_log = np.empty((3,buffer_size))*nan
pd_log = np.empty((3,buffer_size))*nan
pd_des_log = np.empty((3,buffer_size))*nan
pdd_des_log = np.empty((3,buffer_size))*nan

error_o_log = np.empty((3,buffer_size))*nan
tau_log = np.empty((5,buffer_size))*nan
time_log =  np.empty((buffer_size))*nan

q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

q_des  = conf.q0     # joint reference position
qd_des = zero        # joint reference velocities
qdd_des = zero       # joint reference acceleration

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)

# Compute initial end effector position and velocity from q0
p0 = robot.framePlacement(conf.q_d, frame_ee, True).translation + np.array([0.0, 0.0, 0.0])
pd0 = np.array([ 0.0, 0.0, 0.0])
pdd0 = np.array([ 0.0, 0.0, 0.0])

# initialize actual variables
p = p0
pd = pd0
pdd = pdd0

# initialize reference variables
p_des = p0
pd_des = zero_cart
pdd_des = zero_cart

########################
# CONTROLL LOOP
########################

while True:
    
    # # Sinusoidal reference generation for the end-effector
    # p_des  = np.array([1, 2, 1])  + np.multiply(conf.amp, np.sin(two_pi_f*time + conf.phi))
    # pd_des = np.multiply(two_pi_f_amp , np.cos(two_pi_f*time + conf.phi))
    # pdd_des = np.multiply(two_pi_f_squared_amp , -np.sin(two_pi_f*time + conf.phi))
    # # Set constant reference after a while
    # if time >= conf.exp_duration_sin:
    #     p_des  = np.array([1, 2, 1])
    #     pd_des = pd0
    #     pdd_des = pdd0
        
    # Step reference generation for the end effector
    # if time > 2.0:
    #     p_des = p0 + np.array([ -3.0, 1.0, 0.5])
    #     pd_des =  zero_cart
    #     pdd_des = zero_cart
    # else:
    #     p_des = p0
    #     pd_des =  zero_cart
    #     pdd_des = zero_cart

    # Constant reference
    p_des = p0
    p_des = np.array([1, 2, 1])
    pd_des =  zero_cart
    pdd_des = zero_cart

    if time >= conf.exp_duration:
        break
                            
    robot.computeAllTerms(q, qd) 
    # joint space inertia matrix                
    M = robot.mass(q, False)
    # bias terms                
    h = robot.nle(q, qd, False)
    #gravity terms                
    g = robot.gravity(q)
    
    # compute the Jacobian of the end-effector in the world frame
    J6 = robot.frameJacobian(q, frame_ee, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
    # take first 3 rows of J6 cause we have a point contact            
    J = J6[:3,:] 
    # compute  the end-effector acceleration due to joint velocity Jdot*qd         
    Jdqd = robot.frameClassicAcceleration(q, qd, None, frame_ee).linear    
    # compute frame end effector position and velocity in the WF   
    p = robot.framePlacement(q, frame_ee).translation  
    pd = J.dot(qd)  

    M_inv = np.linalg.inv(M)
    # Moore-penrose pseudoinverse  A^# = (A^T*A)^-1 * A^T with A = J^T
    JTpinv = np.linalg.inv(J.dot(J.T)).dot(J)
    
    # joint space inertia matrix reflected at the end effector (J*M^-1*Jt)^-1
    lambda_= np.linalg.inv(J.dot(M_inv).dot(J.T))  # J should be full row rank  otherwise add a damping term
     
    # Null space projector I - (JTpinv )^-1 * JTpinv => I  - JT *JTpiv
    N = eye(5)-J.T.dot(JTpinv)

    # null space torques (postural task)
    tau0 = conf.Kp_postural*(conf.null_space_task-q) - conf.Kd_postural*qd
    tau_null = N.dot(tau0)
     
    # Cartesian space inverse dynamics with postural task
    F_des = pdd_des + conf.Kx.dot(p_des-p) + conf.Dx.dot(pd_des-pd)
    mu =  JTpinv.dot(h) -lambda_.dot(Jdqd)
    tau = J.T.dot(lambda_.dot(F_des) + mu) + tau_null                 
    
    # SIMULATION of the forward dynamics
    qdd = M_inv.dot(tau - h)    
    
    # Forward Euler Integration    
    qd = qd + qdd*conf.dt
    q = q + qd*conf.dt + 0.5*conf.dt*conf.dt*qdd
				    
    # Log Data into a vector
    time_log[log_counter] = time
    p_log[:,log_counter] = p
    p_des_log[:,log_counter] = p_des
    pd_log[:,log_counter] = pd
    pd_des_log[:,log_counter] = pd_des
    tau_log[:,log_counter] = tau

    log_counter+=1  
    # update time
    time = time + conf.dt                  
    
    # plot ball at the end-effector
    ros_pub.add_marker(p)
    ros_pub.add_marker(p_des, color="blue")
    # publish joint variables
    ros_pub.publish(robot, q, qd, tau)                   
    tm.sleep(conf.dt*conf.SLOW_FACTOR)
    
    # stops the while loop if  you prematurely hit CTRL+C                    
    if ros_pub.isShuttingDown():
        print ("Shutting Down")                    
        break;            
ros_pub.deregister_node()
      
#plot position
plotEndeff('ee position', 1,time_log, p_log, p_des_log)
plotEndeff('velocity', 2, time_log, pd_log, pd_des_log)  
plt.show(block=True)
