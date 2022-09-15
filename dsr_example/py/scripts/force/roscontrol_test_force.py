#!/usr/bin/env python3
import rospy
import os
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from copy import copy
from math import sin, cos, tan
from dsr_msgs.msg import ServoJStream, ServoJRTStream, SpeedJRTStream, SpeedJRTStream, RobotStateRT, TorqueRTStream
from trajectory_generate import Trajectory
from time import time
import modern_robotics as mr
import pandas as pd
import tf

ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m0609"
from DSR_ROBOT import *
__dsr__id = ROBOT_ID
__dsr__model = ROBOT_MODEL


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
moment_arm_mr = rospy.Publisher("/dsr01m0609/momemt_arm_mr", Float32MultiArray, queue_size=1)
wrench_mr = rospy.Publisher("/dsr01m0609/wrench_mr", Float32MultiArray, queue_size=1)
mmt_arm_mr = Float32MultiArray(); mmt_arm_mr.data = [0.0]*2
wrench_mr = Float32MultiArray(); wrench_mr.data = [0.0]*6

moment_arm = rospy.Publisher("/dsr01m0609/momemt_arm", Float32MultiArray, queue_size=1)
wrench = rospy.Publisher("/dsr01m0609/wrench", Float32MultiArray, queue_size=1)
mmt_arm = Float32MultiArray(); mmt_arm.data = [0.0]*2
wrench_ = Float32MultiArray(); wrench_.data = [0.0]*6
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
Js_dot = np.zeros((6*6,6))
deg2rad = 3.141592/180
qint_error=np.zeros((6,1))

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

M_a_ = np.diag([2,2,2,0.5,0.5,0.25])
D_a_ = np.diag([15,15,15,5,5,5])

V_d = np.array([0,0,0,0,0,0])
X_d = np.array([350,361,280,45,-10,0])

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
    tor_ext_off = [1.60, 1.5, 0.85, 0.9, -1.7, 0.003]
    pose_tcp = [readdata.data.actual_tcp_position[0],readdata.data.actual_tcp_position[1],readdata.data.actual_tcp_position[2],readdata.data.actual_tcp_position[3],readdata.data.actual_tcp_position[4],readdata.data.actual_tcp_position[5]]
    T = tf.transformations.euler_matrix(pose_tcp[3],pose_tcp[4],pose_tcp[5])
    T[0:3,3] = pose_tcp[0:3]

    tor_ext = alpha*(np.array(tor_ext_tmp)-np.array(tor_ext_off)) + (1-alpha)*np.array(buf_tor_ext)
    buf_tor_ext=tor_ext
    J_b_mr[0:3,:] = mr.JacobianBody(Blist,q)[3:6]
    J_b_mr[3:6,:] = mr.JacobianBody(Blist,q)[0:3]
    J_s_mr[0:3,:] = mr.JacobianSpace(Slist,q)[3:6]
    J_s_mr[3:6,:] = mr.JacobianSpace(Slist,q)[0:3]
    J_s= np.array([readdata.data.jacobian_matrix[0].data, readdata.data.jacobian_matrix[1].data, readdata.data.jacobian_matrix[2].data, readdata.data.jacobian_matrix[3].data, readdata.data.jacobian_matrix[4].data, readdata.data.jacobian_matrix[5].data])
    J_b= mr.Adjoint(mr.TransInv(mr.FKinSpace(T_0,Slist,q)))@J_s
    for i in range(0,6):
        for j in range(0,6):
            Js_dot[6*i:6*i+6,j] = mr.ad(J_s[:,i])@J_s[:,j]

    wrench_ext_mr = np.linalg.pinv(np.transpose(J_b_mr))@tor_ext
    wrench_mr = wrench_ext_mr

    wrench_ext = np.linalg.pinv(np.transpose(J_b))@tor_ext
    wrench_ = wrench_ext

    mmt_arm_mr.data = [max(min(wrench_ext_mr[5]/max(min(wrench_ext_mr[0],10),-10),0.1),-0.1), min(max(-wrench_ext_mr[4]/max(min(wrench_ext_mr[0],10),-10),-0.1),0.1)]
    mmt_arm.data = [max(min(wrench_ext[5]/max(min(wrench_ext[0],10),-10),0.1),-0.1), min(max(-wrench_ext[4]/max(min(wrench_ext[0],10),-10),-0.1),0.1)]
    # print(Jb_dot)
    M_=np.array([readdata.data.mass_matrix[0].data, readdata.data.mass_matrix[1].data, readdata.data.mass_matrix[2].data, readdata.data.mass_matrix[3].data, readdata.data.mass_matrix[4].data, readdata.data.mass_matrix[5].data])

    C_=np.array([readdata.data.coriolis_matrix[0].data, readdata.data.coriolis_matrix[1].data, readdata.data.coriolis_matrix[2].data, readdata.data.coriolis_matrix[3].data, readdata.data.coriolis_matrix[4].data, readdata.data.coriolis_matrix[5].data])

    G_=np.array(tor_g)

    H_=C_@q_dot+G_

    Gamma = np.linalg.pinv(np.transpose(J_s))@M_@np.linalg.pinv(J_s)

    # eta = np.linalg.pinv(np.transpose(J_b))@H_-Gamma@Jb_dot@np.linalg.pinv(J_b)@


    if motion==1 and traj_flag[0]==0:
        # trj_q=[1.5709,0.7,1.5709,0,-0.7,0]
        trj_q=np.array([-45,10,130,0,-50,0])*deg2rad
        # trj_q=np.array([0,0,0,0,0,0])*deg2rad
        # motion+=1
        motion=0
        traj_flag=[1]*6
    elif motion==2 and traj_flag[0]==0:
        trj_q=np.array([0,0,0,0,0,0])*deg2rad
        # motion+=1
        motion=1
        traj_flag=[1]*6
    elif motion==3 and traj_flag[0]==0:
        trj_q=np.array([-1.5709,-0.7,0,0,0,0])*deg2rad
        motion+=1
        traj_flag=[1]*6
    elif motion==4 and traj_flag[0]==0:
        trj_q=np.array([0.0,0.0,0,0,0,0])*deg2rad
        motion=1
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


    Kp=np.array([2400,1300,1300,900,800,800])
    Kd=np.array([12,6,6,2,2,2])

    fc = [11, 9, 5, 4, 4, 3.3]
    fb = [74, 66.24, 25.33, 10.78, 11.866, 10.197]



    V_d_dot = np.linalg.pinv(M_a_)@(-D_a_@V_d+wrench_ext+np.array([0,0,0,0,0,0]))

    V_d = V_d + V_d_dot*(1/10)

    X_d = X_d + V_d*(1/10)
    # Kp=np.array([1000,700,700,450,400,400])
    # Kd=np.array([6,5,5,3,3,2])
    # print(qd_ddot)
    #print("qd:",qd)
    #print("q:",q)
    q_error = np.array(qd)-np.array(q)

    qdot_error = np.array(qd_dot)-np.array(q_dot)
    # print(qdot_error)
    # qint_error = qint_error+q_error*(1/rate)
    # print(qint_error)
    # cmd_tor.tor = M_@(np.array(qd_ddot)+Kp@q_error+Ki@(qint_error)+Kd@qdot_error)+H_
    #print(np.array(qd_ddot)+Kp@q_error+Kd@qdot_error)
    # cmd_tor.tor = M_@(np.array(qd_ddot)+Kp@q_error+Kd@qdot_error)+H_+tor_ext
    #print(cmd_tor)
    # print(H_)
    # print(M_)
    # """
    for i in range(6):
        if i == 0:
            # cmd_tor.tor[i] = 350*(qd[i]-q[i])+3*(qd_dot[i]-q_dot[i])+tor_g[i]+3.5*tor_ext[i]
            cmd_tor.tor[i] = 850*(qd[i]-q[i])+80*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
        elif i == 1:
            # cmd_tor.tor[i] = 400*(qd[i]-q[i])+3.2*(qd_dot[i]-q_dot[i])+tor_g[i]+3.5*tor_ext[i]
            cmd_tor.tor[i] = 850*(qd[i]-q[i])+80*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
        elif i == 2:
            # cmd_tor.tor[i] = 400*(qd[i]-q[i])+3.2*(qd_dot[i]-q_dot[i])+tor_g[i]+3.5*tor_ext[i]
            cmd_tor.tor[i] = 750*(qd[i]-q[i])+70*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
        elif i == 3:
            # cmd_tor.tor[i] = 200*(qd[i]-q[i])+1.9*(qd_dot[i]-q_dot[i])+tor_g[i]+4*tor_ext[i]
            cmd_tor.tor[i] = 400*(qd[i]-q[i])+30*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
        elif i == 4:
            # cmd_tor.tor[i] = 200*(qd[i]-q[i])+1.9*(qd_dot[i]-q_dot[i])+tor_g[i]+5*tor_ext[i]
            cmd_tor.tor[i] = 500*(qd[i]-q[i])+35*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
        elif i == 5:
            # cmd_tor.tor[i] = 150*(qd[i]-q[i])+1.5*(qd_dot[i]-q_dot[i])+tor_g[i]+6*tor_ext[i]
            cmd_tor.tor[i] = 400*(qd[i]-q[i])+30*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
    # """
    command.publish(cmd_tor)
    #print(M_)
    #print(M_@q_dot)
    #print(C_)
    #print(G_)
    #print(np.linalg.pinv(J_b))
    os.system('cls' if os.name == 'nt' else 'clear')
    print(mmt_arm_mr.data)
    print(wrench_ext_mr)
    print(mmt_arm.data)
    print(wrench_ext)
    print(tor_ext)
    print(V_d)
    print(X_d)


    moment_arm_mr.publish(mmt_arm_mr)
    moment_arm.publish(mmt_arm)
    # print(cmd_tor)
    # print("Jb : ",J_b)
    # print("Jb_mr : ",J_b_mr)


	#print(cmd_tor.tor)
    # 10Hz
    governor.sleep()
