#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from copy import copy
from math import sin, cos, tan, pi
from dsr_msgs.msg import ServoJStream, ServoJRTStream, SpeedJRTStream, SpeedJRTStream, RobotStateRT, TorqueRTStream 
from dsr_msgs.srv import SetDesiredForce, TaskComplianceCtrl, MoveLine, MoveJoint
from pvnet_ros.srv import Obj_6d_pose
from trajectory_generate import Trajectory
from time import time
import numpy as np
import tf

from tasho import task_prototype_rockit as tp
from tasho import input_resolution, world_simulator
from tasho import robot as rob
import casadi as cs

ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m0609"
from DSR_ROBOT import *
__dsr__id = ROBOT_ID
__dsr__model = ROBOT_MODEL

plane_consideration = True

# ROS boilerplate
rospy.init_node('roscontrol_test')

#Different OCP options
robot_choice = 'm0609' #'kinova'#'iiwa7' #

robot = rob.Robot(robot_choice)


drt_task_cmplt_ctrl=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/force/task_compliance_ctrl', TaskComplianceCtrl)
drt_release_cmplt = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/force/release_compliance_ctrl', ReleaseComplianceCtrl)
drt_set_desired_force = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/force/set_desired_force', SetDesiredForce)
drt_release_force = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/force/release_force', ReleaseForce)

drt_read = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/read_data_rt', ReadDataRT)

drt_movej = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_joint', MoveJoint)
drt_movel = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_line', MoveLine)

pvnet_read = rospy.ServiceProxy('/obj_6d_pose', Obj_6d_pose)

radius = 0.0; mode = 0; blendType = 0; syncType = 0; ref = 0

Tec = [[-0.0146784, 0.999722, -0.0184761, -0.0805974],\
        [-0.999891, -0.0146492, 0.00171688, 0.0119686],\
        [0.00144574, 0.0184993, 0.999828, -0.00446265],\
        [0, 0, 0, 1]]

readdata=drt_read()
deg2rad = pi/180
rad2deg = 180/pi
print("setup has done")



while True:
    # Init job position cmd
    vel_j = 40.0; acc_j = 60.0; time_j = 3.0
    # pos_j = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # drt_movej(pos_j,vel_j,acc_j,time_j,radius,mode,blendType,syncType)
    pos_j = [0.0, -10.0, 120.0, 0.0, -20.0, 0.0]
    drt_movej(pos_j,vel_j,acc_j,time_j,radius,mode,blendType,syncType)

    # Align Target position
    pos_l = [583.5, -10.22, 211.42, 180.0, 10.0, 180.0]
    vel_l = [40.0, 40.0]; acc_l =  [50.0,50.0]; time_l = 5.0 
    drt_movel(pos_l, vel_l, acc_l, time_l,radius,ref,mode,blendType,syncType)

    rospy.sleep(1)
    readdata=drt_read()
    q = [readdata.data.actual_joint_position[0]*deg2rad,readdata.data.actual_joint_position[1]*deg2rad,readdata.data.actual_joint_position[2]*deg2rad\
        ,readdata.data.actual_joint_position[3]*deg2rad,readdata.data.actual_joint_position[4]*deg2rad,readdata.data.actual_joint_position[5]*deg2rad]
    pvnet_pos = pvnet_read()
    Tcd = tf.transformations.quaternion_matrix([pvnet_pos.obj_6d_pose.orientation.x, pvnet_pos.obj_6d_pose.orientation.y, pvnet_pos.obj_6d_pose.orientation.z, pvnet_pos.obj_6d_pose.orientation.w])
    Tcd = Tcd@tf.transformations.euler_matrix(0,0,-pi/2)
    Tcd[0:3,3] = [pvnet_pos.obj_6d_pose.position.x,pvnet_pos.obj_6d_pose.position.y,pvnet_pos.obj_6d_pose.position.z]
    Ted = np.array(robot.fk(q)[5])@Tec@Tcd

    print(robot.fk(q)[5])
    print('Tcd :',Tcd)
    print('Ted :',Ted)


    # Align Target position
    # pos_l = [643.5, -10.22, 251.42, 180.0, 10.0, 180.0]
    pos_l[0:3] = (Ted[0:3,3]+np.array([-0.0,0,-0.005]))*1000.0; pos_l[3:] = np.array(tf.transformations.euler_from_matrix(Ted))*rad2deg+np.array([0,-10,0])
    pos_l_tmp = pos_l
    vel_l = [40.0, 40.0]; acc_l =  [50.0,50.0]; time_l = 5.0 
    drt_movel(pos_l, vel_l, acc_l, time_l,radius,ref,mode,blendType,syncType)
    
    

    if plane_consideration == True:
        # Set compliance & force
        stx_cmp = [500.0, 500.0, 500.0, 100.0, 100.0, 100.0]; dir_fd = [1,0,0,0,0,0]; fd = [20.0,0,0,0,0,0]     # Without load
        # stx_cmp = [500.0, 500.0, 500.0, 100.0, 100.0, 100.0]; dir_fd = [1,0,1,0,0,0]; fd = [40.0,0,25,0,0,0]  # With 2.5kg load
        drt_task_cmplt_ctrl(stx_cmp,ref,0.0)
        drt_set_desired_force(fd,dir_fd,ref,0.0,mode)

        # Move to target
        # pos_l = [643.5, -10.22, 251.42, 180.0, 10.0, 180.0] # target
        pos_l = pos_l+np.array([0.5,0,0,0,10,0])
        # pos_l = [723.5, -10.22, 251.42, 180.0, 10.0, 180.0]
        drt_movel(pos_l, vel_l, acc_l, 5,radius,ref,mode,blendType,syncType)

        # Set compliance & force
        drt_release_cmplt(); drt_release_force();
        stx_cmp = [500.0, 500.0, 500.0, 100.0, 100.0, 100.0]; dir_fd = [1,0,0,0,0,0]; fd = [50.0,0,0,0,0,0]     # Without load
        # stx_cmp = [500.0, 500.0, 500.0, 100.0, 100.0, 100.0]; dir_fd = [1,0,1,0,0,0]; fd = [40.0,0,25,0,0,0]  # With 2.5kg load
        drt_task_cmplt_ctrl(stx_cmp,ref,0.0)
        drt_set_desired_force(fd,dir_fd,ref,0.0,mode)

        # Move to target
        # pos_l = [643.5, -10.22, 251.42, 180.0, 10.0, 180.0] # target
        pos_l = pos_l+np.array([0.4,0,0,0,-10,0])
        # pos_l = [723.5, -10.22, 251.42, 180.0, 10.0, 180.0]
        drt_movel(pos_l, vel_l, acc_l, 15,radius,ref,mode,blendType,syncType)

    else:
        # Set compliance & force
        stx_cmp = [500.0, 500.0, 500.0, 100.0, 100.0, 100.0]; dir_fd = [1,0,0,0,0,0]; fd = [50.0,0,0,0,0,0]     # Without load
        # stx_cmp = [500.0, 500.0, 500.0, 100.0, 100.0, 100.0]; dir_fd = [1,0,1,0,0,0]; fd = [40.0,0,25,0,0,0]  # With 2.5kg load
        drt_task_cmplt_ctrl(stx_cmp,ref,0.0)
        drt_set_desired_force(fd,dir_fd,ref,0.0,mode)

        # Move to target
        # pos_l = [643.5, -10.22, 251.42, 180.0, 10.0, 180.0] # target
        pos_l = pos_l+np.array([0.5,0,0,0,0,0])
        # pos_l = [723.5, -10.22, 251.42, 180.0, 10.0, 180.0]
        drt_movel(pos_l, vel_l, acc_l, 15,radius,ref,mode,blendType,syncType)


    # Unplug
    drt_release_cmplt(); drt_release_force();
    pos_l = pos_l_tmp+np.array([-0.2,0,0,0,0,0])
    drt_movel(pos_l, vel_l, acc_l, time_l,radius,ref,mode,blendType,syncType)

    