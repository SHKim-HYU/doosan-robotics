#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from copy import copy
from math import sin, cos, tan, pi
from geometry_msgs.msg import PoseStamped
from dsr_msgs.msg import ServoJStream, ServoJRTStream, SpeedJRTStream, SpeedJRTStream, RobotStateRT, TorqueRTStream 
from dsr_msgs.srv import SetDesiredForce, TaskComplianceCtrl, MoveLine, MoveJoint, MoveSpiral, SetSingularityHandling
from trajectory_generate import Trajectory
from time import time
import numpy as np
import tf
import modern_robotics as mr

from multiprocessing import Process, Manager

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
_x = manager.dict()
_x_d = manager.dict()
_global_flag = manager.dict()
_task_flag = manager.dict()

_q['q'] = [0.0]*9; _q['q_dot'] = [0.0]*9; _q['trq_g'] = [0.0]*9; _q['mpc_q']=[0.0]*9; _q['mpc_q_dot']=[0.0]*9;
_q['trq_ext'] = [0.0]*6;
_q_d['qd'] = [0.0]*9; _q_d['qd_dot'] = [0.0]*9; _q_d['qd_ddot'] = [0.0]*9;

_x['pos'] = [0.0]*3; _x['ori'] = [0.0]*4
_x['wrench_ext'] = [0.0]*6;

_x_d['pos_base'] = [0.0]*3; _x_d['ori_base'] = [0.0]*4;
_x_d['pos'] = [0.0]*3; _x_d['ori'] = [0.0]*4;
_x_d['pos_ed'] = [0.0]*3; _x_d['ori_ed'] = [0.0]*4;
_x_d['pos_aruco'] = [0.0]*3; _x_d['ori_aruco'] = [0.0]*4;
_global_flag['MPC_Fail'] = False; _global_flag['OCP_Solved'] = False; _global_flag['Interpolated'] = False; _global_flag['OCP_Solved_mobile'] = False; _global_flag['Interpolated_mobile'] = False;
_global_flag['isHoming'] = False; _global_flag['initial_data'] = False; _global_flag['trajflag'] = [0]*6;
_global_flag['isArrived'] = False; _global_flag['base_flag'] = False; _global_flag['PVNet_received'] = False; _global_flag['buf_mobile']=3;


_task_flag['Task_Transition'] = False;
_task_flag['Task_Robot'] = 0 # 0: Mobile only, 1: Whole-body, 2: Manipulator only


def m0609_cmd_run():
    deg2rad = 3.141592/180
    rad2deg = 180/3.141592
    m0609_frq = 200

    init_time=time.time()
    qd=[0]*6
    qd_dot=[0]*6
    qd_ddot=[0]*6
    
    ### Joint torque sensor
    tor_ext_tmp=[0]*6
    buf_tor_ext=[0]*6
    tor_ext=[0]*6
    frq_cut_off=5
    alpha=(frq_cut_off*(1/m0609_frq))/(1+frq_cut_off*(1/m0609_frq));

    Js_dot = np.zeros((6*6,6))
    qint_error=np.zeros((6,1))
    
    ### Define for trajectory
    traj = Trajectory(6)
    traj_flag = [0]*6
    motion = 1
    target_q = [0]*6
    
    ### Define for Friction comp.
    fc = [11, 9, 5, 4, 4, 3.3]
    fb = [74, 66.24, 25.33, 10.78, 11.866, 10.197]

    rospy.init_node('m0609_cmd_node')

    rospy.wait_for_service('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/read_data_rt')
    try:
        drt_read = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/read_data_rt', ReadDataRT)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    # works with the default example only because I'm not dragging in their libraries to try and parameterize this.

    print("setup has done")

    command = rospy.Publisher("/dsr01m0609/torque_rt_stream", TorqueRTStream, queue_size=1)
    
    cmd_tor=TorqueRTStream()
    cmd_tor.tor = [0]*6
    cmd_tor.time = 1/m0609_frq

    

    ### M0609 Kinematic parameters based on URDF file for MR
    link_01 = np.array([0, 0, 0.135]); link_12 = np.array([0, -0.0062, 0]); link_23 = np.array([0, 0, 0.411]);
    link_34 = np.array([0, 0, 0.368]); link_45 = np.array([0, 0, 0]); link_56 = np.array([0, 0, 0.121]); link_6E = np.array([-0.012, 0, 0.2205])
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

    rate = rospy.Rate(m0609_frq)
    
    
    ### realtime loop start
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


        # print(Jb_dot)
        M_=np.array([readdata.data.mass_matrix[0].data, readdata.data.mass_matrix[1].data, readdata.data.mass_matrix[2].data, readdata.data.mass_matrix[3].data, readdata.data.mass_matrix[4].data, readdata.data.mass_matrix[5].data])

        C_=np.array([readdata.data.coriolis_matrix[0].data, readdata.data.coriolis_matrix[1].data, readdata.data.coriolis_matrix[2].data, readdata.data.coriolis_matrix[3].data, readdata.data.coriolis_matrix[4].data, readdata.data.coriolis_matrix[5].data])

        G_=np.array(tor_g)

        print(tor_g)

        if motion==1 and traj_flag[-1]==0:
            target_q=[0.0*deg2rad, 0.0*deg2rad, 0.0*deg2rad, 0.0*deg2rad, 0.0*deg2rad, 0.0*deg2rad]
            motion+=1
            traj_flag=[1]*6
        elif motion==2 and traj_flag[-1]==0:
            target_q=[-10.0*deg2rad, -10.0*deg2rad, 20.0*deg2rad, -30.0*deg2rad, 10.0*deg2rad, 20.0*deg2rad]
            motion+=1
            traj_flag=[1]*6
        elif motion==3 and traj_flag[-1]==0:
            target_q=[-25.0*deg2rad, 0.0*deg2rad, 10.0*deg2rad, -50.0*deg2rad, 20.0*deg2rad, 40.0*deg2rad]
            motion+=1
            traj_flag=[1]*6
        elif motion==4 and traj_flag[-1]==0:
            target_q=[-50.0*deg2rad, 50.0*deg2rad, 50.0*deg2rad, 50.0*deg2rad, 50.0*deg2rad, 50.0*deg2rad]
            motion+=1
            traj_flag=[1]*6
        elif motion==5 and traj_flag[-1]==0:
            target_q=[-30.0*deg2rad, 10.0*deg2rad, 30.0*deg2rad, -20.0*deg2rad, 10.0*deg2rad, 60.0*deg2rad]
            motion+=1
            traj_flag=[1]*6
        elif motion==6 and traj_flag[-1]==0:
            target_q=[-20.0*deg2rad, 20.0*deg2rad, 40.0*deg2rad, 20.0*deg2rad, 0.0*deg2rad, 90.0*deg2rad]
            motion=1
            traj_flag=[1]*6
        print(target_q)
            
        for i in range(6):
            if traj_flag[i]==1:
                traj.SetPolynomial5th(i,q[i],target_q[i],g_time,5.0)
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
         
        print(cmd_tor)   
        command.publish(cmd_tor)


        rate.sleep()    

if __name__=='__main__':

    m0609_cmd_task = Process(target=m0609_cmd_run, args=())
   
    try:
        m0609_cmd_task.start()

        m0609_cmd_task.join()

    except KeyboardInterrupt:
        m0609_cmd_task.terminate()
