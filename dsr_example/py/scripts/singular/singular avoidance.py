#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from copy import copy
from math import sin, cos, tan
from dsr_msgs.msg import ServoJStream, ServoJRTStream, SpeedJRTStream, SpeedJRTStream, RobotStateRT, TorqueRTStream
from trajectory_generate import Trajectory
from time import time
import numpy as np

from multiprocessing import Process, Manager
import tf
import modern_robotics as mr

### DSR namespace

ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m0609"
from DSR_ROBOT import *
__dsr__id = ROBOT_ID
__dsr__model = ROBOT_MODEL

#############################################################################
################## Manager for global data multiprocessing ##################
#############################################################################

manager = Manager()
_q = manager.dict()
_q_d = manager.dict()

_q['q_r'] = [0.0]*6; _q['q_r_dot'] = [0.0]*6; _q['trq_r_g'] = [0.0]*6; 
_q['q_n'] = [0.0]*6; _q['q_n_dot'] = [0.0]*6; _q['trq_n_g'] = [0.0]*6; 
_q_d['qd'] = [0.0]*6; _q_d['qd_dot'] = [0.0]*6; _q_d['qd_ddot'] = [0.0]*6; _q['trq_ext'] = [0.0]*6; _q['wrench_ext'] = [0.0]*6;
_q_d['qd_itp']=[[0.0]*horizon_size]*6
_q_d['qd_dot_itp']=[[0.0]*horizon_size]*6
_q_d['qd_ddot_itp']=[[0.0]*horizon_size]*6


def nominal_run():
    rate = 1000
    rospy.init_node('nominal_node')
    nominal_rate = rospy.Rate(rate)

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

    while not rospy.is_shutdown():


        nominal_rate.sleep()

def m0609_run():

    ### Sellect the controller options: position and velocity controller are not implemented yet (those make some purterbation during the execution)
    control_type = "gravity" #"gravity" #"velocity" #"position" #

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

    if control_type=="position":
        command = rospy.Publisher("/dsr01m0609/servoj_rt_stream", ServoJRTStream, queue_size=1)
    elif control_type=="velocity":
        command = rospy.Publisher("/dsr01m0609/speedj_rt_stream", SpeedJRTStream, queue_size=1)
    elif control_type=="torque" or  control_type == "gravity":
        command = rospy.Publisher("/dsr01m0609/torque_rt_stream", TorqueRTStream, queue_size=1)
    governor=rospy.Rate(rate)


    if control_type=="position":
        cmd_pos=ServoJRTStream()
        cmd_pos.time=1/rate
    elif control_type=="velocity":
        cmd_vel=SpeedJRTStream()
        cmd_vel.time=1/rate
    elif control_type == "torque" or control_type == "gravity" :
        cmd_tor=TorqueRTStream()
        cmd_tor.tor = [0]*6
        cmd_tor.time = 1/rate

    init_time=time.time()
    deg2rad = 3.141592/180
    qd=[0]*6
    qd_dot=[0]*6
    qd_ddot=[0]*6
    tor_ext_tmp=[0]*6
    buf_tor_ext=[0]*6
    tor_ext=[0]*6
    frq_cut_off=10
    alpha=(frq_cut_off*(1/rate))/(1+frq_cut_off*(1/rate));

    fc = [11, 9, 5, 4, 4, 3.3]
    fb = [74, 66.24, 25.33, 10.78, 11.866, 10.197]

    readdata=drt_read()
    init_pos = readdata.data.actual_joint_position
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

        if motion==1 and traj_flag[0]==0:
            trj_q=np.array([-90,45,90,0,-45,0])*deg2rad
            # trj_q=np.array([-90,45,90,0,-45,0])*deg2rad
    #        trj_q=[90,45,90,0,-45,0]
            # motion+=1
            motion=0
            traj_flag=[1]*6
        elif motion==2 and traj_flag[0]==0:
            trj_q=np.array([-90,0,0,0,0,0])*deg2rad
            motion+=1
            traj_flag=[1]*6
        elif motion==3 and traj_flag[0]==0:
            trj_q=np.array([-100, -10, 20, -30, 10, 20])*deg2rad
            motion+=1
            traj_flag=[1]*6
        elif motion==4 and traj_flag[0]==0:
            trj_q=np.array([-115, 0, 10, -50, 20, 40])*deg2rad
            motion+=1
            traj_flag=[1]*6
        elif motion==5 and traj_flag[0]==0:
            trj_q=np.array([-140, 50, 50, 50, 50, 50])*deg2rad
            motion+=1
            traj_flag=[1]*6
        elif motion==6 and traj_flag[0]==0:
            trj_q=np.array([-120, 10, 30, -20, 10, 60])*deg2rad
            motion+=1
            traj_flag=[1]*6
        elif motion==7 and traj_flag[0]==0:
            trj_q=np.array([-110, 20, 40, 20, 0, 90])*deg2rad
            motion=1
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

        if control_type == "position":
            # generate a signal for the robot to track
            for i in range(6):
                cmd_pos.pos[i] = qd[i]
                cmd_pos.vel[i] = qd_dot[i]
                cmd_pos.acc[i] = qd_ddot[i]

            #publish command
            command.publish(cmd_pos)

            print(cmd_pos.pos)
            print(cmd_pos.vel)
            print(cmd_pos.acc)
            print("")

        elif control_type == "velocity":
            # generate a signal for the robot to track
            for i in range(6):
                cmd_vel.vel[i] = qd_dot[i]
                cmd_vel.acc[i] = qd_ddot[i]

            #publish command
            command.publish(cmd_vel)

            print(cmd_vel.vel)
            print(cmd_vel.acc)
            print(cmd_vel.time)
            print("")

        elif control_type == "torque":


            for i in range(6):
                tor_ext[i] = alpha*tor_ext_tmp[i] + (1-alpha)*buf_tor_ext[i];
                if i == 0:
                    # cmd_tor.tor[i] = 500*(qd[i]-q[i])+30*(qd_dot[i]-q_dot[i])+tor_g[i]+3.5*tor_ext[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
                    cmd_tor.tor[i] = 500*(qd[i]-q[i])+30*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
                elif i == 1:
                    # cmd_tor.tor[i] = 750*(qd[i]-q[i])+70*(qd_dot[i]-q_dot[i])+tor_g[i]+3.5*tor_ext[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
                    cmd_tor.tor[i] = 750*(qd[i]-q[i])+70*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
                elif i == 2:
                    # cmd_tor.tor[i] = 650*(qd[i]-q[i])+60*(qd_dot[i]-q_dot[i])+tor_g[i]+3.5*tor_ext[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
                    cmd_tor.tor[i] = 650*(qd[i]-q[i])+60*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
                elif i == 3:
                    # cmd_tor.tor[i] = 300*(qd[i]-q[i])+20*(qd_dot[i]-q_dot[i])+tor_g[i]+4*tor_ext[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
                    cmd_tor.tor[i] = 300*(qd[i]-q[i])+20*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
                elif i == 4:
                    # cmd_tor.tor[i] = 400*(qd[i]-q[i])+25*(qd_dot[i]-q_dot[i])+tor_g[i]+5*tor_ext[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
                    cmd_tor.tor[i] = 400*(qd[i]-q[i])+25*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
                elif i == 5:
                    # cmd_tor.tor[i] = 300*(qd[i]-q[i])+20*(qd_dot[i]-q_dot[i])+tor_g[i]+6*tor_ext[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
                    cmd_tor.tor[i] = 300*(qd[i]-q[i])+20*(qd_dot[i]-q_dot[i])+tor_g[i]+(fc[i]*np.sign(qd_dot[i])+fb[i]*qd_dot[i])
                buf_tor_ext[i]=tor_ext[i]
            command.publish(cmd_tor)

            # print(cmd_tor.tor)
            print(np.array(qd)-np.array(q))

        elif control_type == "gravity":
            for i in range(6):
                tor_ext[i] = alpha*tor_ext_tmp[i] + (1-alpha)*buf_tor_ext[i];
                cmd_tor.tor[i] = tor_g[i]+2.5*tor_ext[i]
                buf_tor_ext[i]=tor_ext[i]
            command.publish(cmd_tor)

            print(cmd_tor.tor)
        # 10Hz
        governor.sleep()


if __name__ == '__main__':

    m0609_task = Process(target=m0609_run, args=())
    nominal_task = Process(target=nominal_run, args=())

    try:
        m0609_task.start()
        nominal_task.start()

        m0609_task.join()
        nominal_task.join()

    except KeyboardInterrupt:
        m0609_task.terminate()
        nominal_task.terminate()        