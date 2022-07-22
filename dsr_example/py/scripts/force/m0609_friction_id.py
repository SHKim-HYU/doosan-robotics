#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from copy import copy
from math import sin, cos, tan, pi
from dsr_msgs.msg import ServoJStream, ServoJRTStream, SpeedJRTStream, SpeedJRTStream, RobotStateRT, TorqueRTStream
from trajectory_generate import Trajectory
from time import time
import modern_robotics as mr
import pandas as pd
import os

ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m0609"
from DSR_ROBOT import *
__dsr__id = ROBOT_ID
__dsr__model = ROBOT_MODEL

Joint_number = "2"


### Sellect the controller options: position and velocity controller are not implemented yet (those make some purterbation during the execution)
control_type = "torque" #"velocity" #"position" # "gravity" #

traj = Trajectory(6)
traj_flag = [0]*6
traj_res = [0]*6
motion = 1


rate=200
# works with the default example only because I'm not dragging in their libraries to try and parameterize this.


# ROS boilerplate

print("Setting up node roscontrol_test, I will wiggle the second last joint ~10deg each way.")
rospy.init_node('roscontrol_test')


drt_read=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/read_data_rt', ReadDataRT)
drt_write=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/write_data_rt', WriteDataRT)

print("setup has done")

command = rospy.Publisher("/dsr01m0609/torque_rt_stream", TorqueRTStream, queue_size=1)
governor=rospy.Rate(rate)

cmd_tor=TorqueRTStream()
cmd_tor.tor = [0]*6
cmd_tor.time = 1/rate

init_time=time.time()
qd=[0]*6
qd_dot=[0]*6
qd_ddot=[0]*6
tor_ext_tmp=[0]*6
buf_tor_ext=[0]*6
tor_ext=[0]*6
frq_cut_off=5
alpha=(frq_cut_off*(1/rate))/(1+frq_cut_off*(1/rate));
Jb_dot = np.zeros((6*6,6))
deg2rad = 3.141592/180
qint_error=np.zeros((6,1))

readdata=drt_read()
init_pos = [readdata.data.actual_joint_position[0]*deg2rad,readdata.data.actual_joint_position[1]*deg2rad,readdata.data.actual_joint_position[2]*deg2rad\
,readdata.data.actual_joint_position[3]*deg2rad,readdata.data.actual_joint_position[4]*deg2rad,readdata.data.actual_joint_position[5]*deg2rad]
print(init_pos)
print("Commanding a small wiggle")
while not rospy.is_shutdown():
    g_time = time.time()
    readdata=drt_read()
    q = [readdata.data.actual_joint_position[0]*deg2rad,readdata.data.actual_joint_position[1]*deg2rad,readdata.data.actual_joint_position[2]*deg2rad\
    ,readdata.data.actual_joint_position[3]*deg2rad,readdata.data.actual_joint_position[4]*deg2rad,readdata.data.actual_joint_position[5]*deg2rad]
    q_dot = [readdata.data.actual_joint_velocity[0]*deg2rad,readdata.data.actual_joint_velocity[1]*deg2rad,readdata.data.actual_joint_velocity[2]*deg2rad\
    ,readdata.data.actual_joint_velocity[3]*deg2rad,readdata.data.actual_joint_velocity[4]*deg2rad,readdata.data.actual_joint_velocity[5]*deg2rad]
    tor_g = readdata.data.gravity_torque
    tor_ext_tmp = readdata.data.external_joint_torque
    tor_ext_off = [1.324512, 1.249877, 0.5998075, 1.1033858, -1.50536, 0.084]

    tor_ext = alpha*(np.array(tor_ext_tmp)-np.array(tor_ext_off)) + (1-alpha)*np.array(buf_tor_ext)
    buf_tor_ext=tor_ext

    J_b= np.array([readdata.data.jacobian_matrix[0].data, readdata.data.jacobian_matrix[1].data, readdata.data.jacobian_matrix[2].data, readdata.data.jacobian_matrix[3].data, readdata.data.jacobian_matrix[4].data, readdata.data.jacobian_matrix[5].data])
    for i in range(0,6):
        for j in range(0,6):
            Jb_dot[6*i:6*i+6,j] = mr.ad(J_b[:,i])@J_b[:,j]

    # print(Jb_dot)
    M_=np.array([readdata.data.mass_matrix[0].data, readdata.data.mass_matrix[1].data, readdata.data.mass_matrix[2].data, readdata.data.mass_matrix[3].data, readdata.data.mass_matrix[4].data, readdata.data.mass_matrix[5].data])

    C_=np.array([readdata.data.coriolis_matrix[0].data, readdata.data.coriolis_matrix[1].data, readdata.data.coriolis_matrix[2].data, readdata.data.coriolis_matrix[3].data, readdata.data.coriolis_matrix[4].data, readdata.data.coriolis_matrix[5].data])

    G_=np.array(tor_g)

    H_=C_@q_dot+G_

    Gamma = np.linalg.pinv(np.transpose(J_b))@M_@np.linalg.pinv(J_b)

    # eta = np.linalg.pinv(np.transpose(J_b))@H_-Gamma@Jb_dot@np.linalg.pinv(J_b)@


    if motion==0 and traj_flag[0]==0:
        if Joint_number == "1":
            qd[0] = trj_q[0] + 0.2*sin(2*pi*0.1*(g_time-init_time))
            qd_dot[0] = 0.4*pi*0.1*cos(2*pi*0.1*(g_time-init_time))
        elif Joint_number == "2":
            qd[1] = trj_q[1] + 0.2*sin(2*pi*0.1*(g_time-init_time))
            qd_dot[1] = 0.4*pi*0.1*cos(2*pi*0.1*(g_time-init_time))
        elif Joint_number == "3":
            qd[2] = trj_q[2] + 0.2*sin(2*pi*0.1*(g_time-init_time))
            qd_dot[2] = 0.4*pi*0.1*cos(2*pi*0.1*(g_time-init_time))
        elif Joint_number == "4":
            qd[3] = trj_q[3] + 0.2*sin(2*pi*0.1*(g_time-init_time))
            qd_dot[3] = 0.4*pi*0.1*cos(2*pi*0.1*(g_time-init_time))
        elif Joint_number == "5":
            qd[4] = trj_q[4] + 0.2*sin(2*pi*0.1*(g_time-init_time))
            qd_dot[4] = 0.4*pi*0.1*cos(2*pi*0.1*(g_time-init_time))
        elif Joint_number == "6":
            qd[5] = trj_q[5] + 0.2*sin(2*pi*0.1*(g_time-init_time))
            qd_dot[5] = 0.4*pi*0.1*cos(2*pi*0.1*(g_time-init_time))

    if motion==1 and traj_flag[0]==0:
        # trj_q=[1.5709,0.7,1.5709,0,-0.7,0]
        trj_q=[0,0,0,0,0,0]
        motion=0
        # motion=0
        traj_flag=[1]*6


    for i in range(6):
        if traj_flag[i]==1:
            traj.SetPolynomial5th(i,q[i],trj_q[i],g_time,3.0)
            qd[i]=q[i]
            qd_dot[i]=0
            qd_ddot[i]=0
            traj_flag[i]=2

        elif traj_flag[i]==2:
            tmp_res,tmp_flag=traj.Polynomial5th(i,g_time)
            qd[i] = tmp_res[0]
            qd_dot[i] = tmp_res[1]
            qd_ddot[i] = tmp_res[2]
            if tmp_flag == 0:
                traj_flag[i]=0


    fc = [11, 9, 5, 4, 4, 3.3]
    fb = [74, 66.24, 25.33, 10.78, 11.866, 10.197]
    
    for i in range(6):
        if i == 0:
            # cmd_tor.tor[i] = 350*(qd[i]-q[i])+3*(qd_dot[i]-q_dot[i])+tor_g[i]+3.5*tor_ext[i]
            # cmd_tor.tor[i] = 350*(qd[i]-q[i])+3*(qd_dot[i]-q_dot[i])+tor_g[i]
            cmd_tor.tor[i] = 350*(qd[i]-q[i])+3*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
        elif i == 1:
            # cmd_tor.tor[i] = 400*(qd[i]-q[i])+3.2*(qd_dot[i]-q_dot[i])+tor_g[i]+3.5*tor_ext[i]
            # cmd_tor.tor[i] = 400*(qd[i]-q[i])+3.2*(qd_dot[i]-q_dot[i])+tor_g[i]
            cmd_tor.tor[i] = 400*(qd[i]-q[i])+3.2*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
        elif i == 2:
            # cmd_tor.tor[i] = 400*(qd[i]-q[i])+3.2*(qd_dot[i]-q_dot[i])+tor_g[i]+3.5*tor_ext[i]
            # cmd_tor.tor[i] = 400*(qd[i]-q[i])+3.2*(qd_dot[i]-q_dot[i])+tor_g[i]
            cmd_tor.tor[i] = 400*(qd[i]-q[i])+3.2*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
        elif i == 3:
            # cmd_tor.tor[i] = 200*(qd[i]-q[i])+1.9*(qd_dot[i]-q_dot[i])+tor_g[i]+4*tor_ext[i]
            # cmd_tor.tor[i] = 200*(qd[i]-q[i])+1.9*(qd_dot[i]-q_dot[i])+tor_g[i]
            cmd_tor.tor[i] = 200*(qd[i]-q[i])+1.9*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
        elif i == 4:
            # cmd_tor.tor[i] = 200*(qd[i]-q[i])+1.9*(qd_dot[i]-q_dot[i])+tor_g[i]+5*tor_ext[i]
            # cmd_tor.tor[i] = 200*(qd[i]-q[i])+1.9*(qd_dot[i]-q_dot[i])+tor_g[i]
            cmd_tor.tor[i] = 200*(qd[i]-q[i])+1.9*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
        elif i == 5:
            # cmd_tor.tor[i] = 150*(qd[i]-q[i])+1.5*(qd_dot[i]-q_dot[i])+tor_g[i]+6*tor_ext[i]
            # cmd_tor.tor[i] = 150*(qd[i]-q[i])+1.5*(qd_dot[i]-q_dot[i])+tor_g[i]
            cmd_tor.tor[i] = 150*(qd[i]-q[i])+1.5*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
    # """
    command.publish(cmd_tor)

    jnt_trq = {'JT1':[cmd_tor.tor[0]-tor_g[0]],'JT2':[cmd_tor.tor[1]-tor_g[1]],'JT3':[cmd_tor.tor[2]-tor_g[2]],'JT4':[cmd_tor.tor[3]-tor_g[3]]\
    ,'JT5':[cmd_tor.tor[4]-tor_g[3]],'JT6':[cmd_tor.tor[5]-tor_g[4]]}
    df_jt = pd.DataFrame(jnt_trq)
    jnt_vel = {'JV1':[q_dot[0]],'JV2':[q_dot[1]],'JV3':[q_dot[2]],'JV4':[q_dot[3]],'JV5':[q_dot[4]],'JV6':[q_dot[5]]}
    df_jv = pd.DataFrame(jnt_vel)

    if not os.path.exists(Joint_number+'_jt.csv'):
        df_jt.to_csv(Joint_number+'_jt.csv', index=False, mode='w', encoding='utf-8-sig')
    else:
        df_jt.to_csv(Joint_number+'_jt.csv', index=False, mode='a', encoding='utf-8-sig', header=False)

    if not os.path.exists(Joint_number+'_jv.csv'):
        df_jv.to_csv(Joint_number+'_jv.csv', index=False, mode='w', encoding='utf-8-sig')
    else:
        df_jv.to_csv(Joint_number+'_jv.csv', index=False, mode='a', encoding='utf-8-sig', header=False)
    # 10Hz
    governor.sleep()
