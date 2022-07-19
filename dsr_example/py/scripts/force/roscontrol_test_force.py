#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from copy import copy
from math import sin, cos, tan
from dsr_msgs.msg import ServoJStream, ServoJRTStream, SpeedJRTStream, SpeedJRTStream, RobotStateRT, TorqueRTStream
from trajectory_generate import Trajectory
from time import time

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


readdata=drt_read()
init_pos = readdata.data.actual_joint_position
print(init_pos)
print("Commanding a small wiggle")
while not rospy.is_shutdown():
    g_time = time.time()
    readdata=drt_read()
    q = readdata.data.actual_joint_position
    q_dot = readdata.data.actual_joint_velocity
    tor_g = readdata.data.gravity_torque
    tor_ext_tmp = readdata.data.external_joint_torque
    tor_ext_off = [1.324512, 1.249877, 0.5998075, 1.1033858, -1.50536, 0.084]
    
    J_b= np.array([readdata.data.jacobian_matrix[0].data, readdata.data.jacobian_matrix[1].data, readdata.data.jacobian_matrix[2].data, readdata.data.jacobian_matrix[3].data, readdata.data.jacobian_matrix[4].data, readdata.data.jacobian_matrix[5].data])
    
    M_=np.array([readdata.data.mass_matrix[0].data, readdata.data.mass_matrix[1].data, readdata.data.mass_matrix[2].data, readdata.data.mass_matrix[3].data, readdata.data.mass_matrix[4].data, readdata.data.mass_matrix[5].data])
    
    C_=np.array([readdata.data.coriolis_matrix[0].data, readdata.data.coriolis_matrix[1].data, readdata.data.coriolis_matrix[2].data, readdata.data.coriolis_matrix[3].data, readdata.data.coriolis_matrix[4].data, readdata.data.coriolis_matrix[5].data])
    
    G_=np.array(tor_g)

    if motion==1 and traj_flag[0]==0:
        trj_q=[0,0,0,0,0,0]
        motion=0
        traj_flag=[1]*6

    for i in range(6):
        if traj_flag[i]==1:
            traj.SetPolynomial5th(i,q[i],trj_q[i],g_time,3.0)
            qd[i]=q[i]
            traj_flag[i]=2

        elif traj_flag[i]==2:
            tmp_res,tmp_flag=traj.Polynomial5th(i,g_time)
            qd[i] = tmp_res[0]
            qd_dot[i] = tmp_res[1]
            qd_ddot[i] = tmp_res[2]
            if tmp_flag == 0:
                traj_flag[i]=0


    for i in range(6):
        tor_ext[i] = alpha*(tor_ext_tmp[i]-tor_ext_off[i]) + (1-alpha)*buf_tor_ext[i]
        if i == 0:
            cmd_tor.tor[i] = 10*(qd[i]-q[i])+0.1*(qd_dot[i]-q_dot[i])+tor_g[i]+3.5*tor_ext[i]
        elif i == 1:
            cmd_tor.tor[i] = 10*(qd[i]-q[i])+0.1*(qd_dot[i]-q_dot[i])+tor_g[i]+3.5*tor_ext[i]
        elif i == 2:
            cmd_tor.tor[i] = 10*(qd[i]-q[i])+0.1*(qd_dot[i]-q_dot[i])+tor_g[i]+3.5*tor_ext[i]
        elif i == 3:
            cmd_tor.tor[i] = 10*(qd[i]-q[i])+0.1*(qd_dot[i]-q_dot[i])+tor_g[i]+4*tor_ext[i]
        elif i == 4:
            cmd_tor.tor[i] = 10*(qd[i]-q[i])+0.1*(qd_dot[i]-q_dot[i])+tor_g[i]+5*tor_ext[i]
        elif i == 5:
            cmd_tor.tor[i] = 10*(qd[i]-q[i])+0.1*(qd_dot[i]-q_dot[i])+tor_g[i]+6*tor_ext[i]
        buf_tor_ext[i]=tor_ext[i]
    #command.publish(cmd_tor)
    #print(M_)
    #print(M_@q_dot)
    #print(C_)
    #print(G_)
    #print(np.linalg.pinv(J_b))
    print(np.linalg.pinv(np.transpose(J_b))@tor_ext)
	#print(cmd_tor.tor)
    # 10Hz
    governor.sleep()
