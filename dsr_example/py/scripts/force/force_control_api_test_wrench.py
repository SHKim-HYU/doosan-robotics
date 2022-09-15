#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from copy import copy
from math import sin, cos, tan
from dsr_msgs.msg import ServoJStream, ServoJRTStream, SpeedJRTStream, SpeedJRTStream, RobotStateRT, TorqueRTStream 
from dsr_msgs.srv import SetDesiredForce, TaskComplianceCtrl, MoveLine, MoveJoint, ReadDataRT
from trajectory_generate import Trajectory
from time import time
import numpy as np
import modern_robotics as mr

ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m0609"
from DSR_ROBOT import *
__dsr__id = ROBOT_ID
__dsr__model = ROBOT_MODEL


# ROS boilerplate
rospy.init_node('roscontrol_test')
deg2rad = 3.141592/180

drt_task_cmplt_ctrl=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/force/task_compliance_ctrl', TaskComplianceCtrl)
drt_release_cmplt = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/force/release_compliance_ctrl', ReleaseComplianceCtrl)
drt_set_desired_force = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/force/set_desired_force', SetDesiredForce)
drt_release_force = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/force/release_force', ReleaseForce)
drt_read = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/read_data_rt',ReadDataRT)

drt_movej = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_joint', MoveJoint)
drt_movel = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_line', MoveLine)

radius = 0.0; mode = 0; blendType = 0; syncType = 0; ref = 0

readdata=drt_read()

# Torque sensor
alpha=(frq_cut_off*(1/rate))/(1+frq_cut_off*(1/rate));
### M0609 Kinematic parameters based on URDF file
link_01 = np.array([0, 0, 0.135]); link_12 = np.array([0, -0.0062, 0]); link_23 = np.array([0, 0, 0.411]);
link_34 = np.array([0, 0, 0.368]); link_45 = np.array([0, 0, 0]); link_56 = np.array([0, 0, 0.121]); link_6E = np.array([-0.024, 0, 0.0735])
w=np.array([[0,0,1],[0,1,0],[0,1,0],[0,0,1],[0,1,0],[0,0,1]])
L_=np.array([(link_01+link_12)[:], (link_01+link_12+link_23)[:],(link_01+link_12+link_23+link_34)[:],(link_01+link_12+link_23+link_34+link_45)[:]\
,(link_01+link_12+link_23+link_34+link_45+link_56)[:],(link_01+link_12+link_23+link_34+link_45+link_56+link_6E)[:]])
P_=np.array([link_01[:],(link_01+link_12)[:], (link_01+link_12+link_23)[:],(link_01+link_12+link_23+link_34)[:],(link_01+link_12+link_23+link_34+link_45)[:]\
,(link_01+link_12+link_23+link_34+link_45+link_56)[:]])
v = np.array([-mr.VecToso3(w[0,:])@P_[0,:],-mr.VecToso3(w[1,:])@P_[1,:],-mr.VecToso3(w[2,:])@P_[2,:],-mr.VecToso3(w[3,:])@P_[3,:],-mr.VecToso3(w[4,:])@P_[4,:],-mr.VecToso3(w[5,:])@P_[5,:]])
Slist = np.transpose(np.array([np.append(w[0,:],v[0,:]),np.append(w[1,:],v[1,:]),np.append(w[2,:],v[2,:]),np.append(w[3,:],v[3,:]),np.append(w[4,:],v[4,:]),np.append(w[5,:],v[5,:])]))
T_0 = np.array([[1,0,0,L_[-1,0]],[0,1,0,L_[-1,1]],[0,0,1,L_[-1,2]],[0,0,0,1]])
T_0[0:3,0:3] = tf.transformations.euler_matrix(0,-100*deg2rad,0)[0:3,0:3]
Blist = mr.Adjoint(mr.TransInv(T_0))@Slist
J_b_mr = np.zeros((6,6))
J_s_mr = np.zeros((6,6))




print("setup has done")

while True:
    # Data read
    readdata=drt_read()
    q = [readdata.data.actual_joint_position[0]*deg2rad,readdata.data.actual_joint_position[1]*deg2rad,readdata.data.actual_joint_position[2]*deg2rad\
    ,readdata.data.actual_joint_position[3]*deg2rad,readdata.data.actual_joint_position[4]*deg2rad,readdata.data.actual_joint_position[5]*deg2rad]
    q_dot = [readdata.data.actual_joint_velocity[0]*deg2rad,readdata.data.actual_joint_velocity[1]*deg2rad,readdata.data.actual_joint_velocity[2]*deg2rad\
    ,readdata.data.actual_joint_velocity[3]*deg2rad,readdata.data.actual_joint_velocity[4]*deg2rad,readdata.data.actual_joint_velocity[5]*deg2rad]
    
    # LPF torque sensor data
    tor_ext_tmp = readdata.data.external_joint_torque
    tor_ext_off = [1.60, 1.5, 0.85, 0.9, -1.7, 0.003]
    tor_ext = alpha*(np.array(tor_ext_tmp)-np.array(tor_ext_off)) + (1-alpha)*np.array(buf_tor_ext)
    buf_tor_ext=tor_ext
    
    # Jacobian M0609
    J_b_mr[0:3,:] = mr.JacobianBody(Blist,q)[3:6]
    J_b_mr[3:6,:] = mr.JacobianBody(Blist,q)[0:3]
    J_s_mr[0:3,:] = mr.JacobianSpace(Slist,q)[3:6]
    J_s_mr[3:6,:] = mr.JacobianSpace(Slist,q)[0:3]

    wrench_ext_mr = np.linalg.pinv(np.transpose(J_b_mr))@tor_ext
    wrench_mr = wrench_ext_mr

    wrech_norm = np.linalg.norm(wrench_mr[0:3],2)

    # Init job position cmd
    vel_j = 40.0; acc_j = 60.0; time_j = 3.0
    pos_j = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    drt_movej(pos_j,vel_j,acc_j,time_j,radius,mode,blendType,syncType)
    pos_j = [0.0, -10.0, 120.0, 0.0, -20.0, 0.0]
    drt_movej(pos_j,vel_j,acc_j,time_j,radius,mode,blendType,syncType)

    # Align Target position
    pos_l = [643.5, -10.22, 251.42, 180.0, 10.0, 180.0]
    vel_l = [40.0, 40.0]; acc_l =  [50.0,50.0]; time_l = 5.0 
    drt_movel(pos_l, vel_l, acc_l, time_l,radius,ref,mode,blendType,syncType)

    # Set compliance & force
    stx_cmp = [500.0, 500.0, 500.0, 100.0, 100.0, 100.0]; dir_fd = [1,0,0,0,0,0]; fd = [20.0,0,0,0,0,0]     # Without load
    # stx_cmp = [500.0, 500.0, 500.0, 100.0, 100.0, 100.0]; dir_fd = [1,0,1,0,0,0]; fd = [40.0,0,25,0,0,0]  # With 2.5kg load
    drt_task_cmplt_ctrl(stx_cmp,ref,0.0)
    drt_set_desired_force(fd,dir_fd,ref,0.0,mode)
    """
    # Move to target
    pos_l = [723.5, -10.22, 251.42, 180.0, 10.0, 180.0]
    drt_movel(pos_l, vel_l, acc_l, 15,radius,ref,mode,blendType,syncType)

    # Unplug
    drt_release_cmplt(); drt_release_force();
    pos_l = [603.5, -10.22, 251.42, 180.0, 10.0, 180.0]
    drt_movel(pos_l, vel_l, acc_l, time_l,radius,ref,mode,blendType,syncType)
    """

    pos_l = [643.5, -10.22, 251.42, 180.0, 10.0, 180.0]
    drt_movel(pos_l, vel_l, acc_l, 15,radius,ref,mode,blendType,syncType)

    # Unplug
    drt_release_cmplt(); drt_release_force();
    pos_l = [603.5, -10.22, 251.42, 180.0, 10.0, 180.0]
    drt_movel(pos_l, vel_l, acc_l, time_l,radius,ref,mode,blendType,syncType)