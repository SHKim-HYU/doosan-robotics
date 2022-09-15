#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from copy import copy
from math import sin, cos, tan
from dsr_msgs.msg import ServoJStream, ServoJRTStream, SpeedJRTStream, SpeedJRTStream, RobotStateRT, TorqueRTStream 
from dsr_msgs.srv import SetDesiredForce, TaskComplianceCtrl, MoveLine, MoveJoint
from trajectory_generate import Trajectory
from time import time
import numpy as np

ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m0609"
from DSR_ROBOT import *
__dsr__id = ROBOT_ID
__dsr__model = ROBOT_MODEL


# ROS boilerplate
rospy.init_node('roscontrol_test')

drt_task_cmplt_ctrl=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/force/task_compliance_ctrl', TaskComplianceCtrl)
drt_release_cmplt = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/force/release_compliance_ctrl', ReleaseComplianceCtrl)
drt_set_desired_force = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/force/set_desired_force', SetDesiredForce)
drt_release_force = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/force/release_force', ReleaseForce)

drt_read = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/read_data_rt', ReadDataRT)

drt_movej = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_joint', MoveJoint)
drt_movel = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_line', MoveLine)

radius = 0.0; mode = 0; blendType = 0; syncType = 0; ref = 0

print("setup has done")



while True:
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
    stx_cmp = [500.0, 500.0, 500.0, 100.0, 100.0, 100.0]; dir_fd = [1,0,0,0,0,0]; fd = [30.0,0,0,0,0,0]     # Without load
    # stx_cmp = [500.0, 500.0, 500.0, 100.0, 100.0, 100.0]; dir_fd = [1,0,1,0,0,0]; fd = [40.0,0,25,0,0,0]  # With 2.5kg load
    drt_task_cmplt_ctrl(stx_cmp,ref,0.0)
    drt_set_desired_force(fd,dir_fd,ref,0.0,mode)



    # Move to target
    pos_l = [723.5, -5.22, 246.42, 180.0, 10.0, 180.0]
    # pos_l = [723.5, -10.22, 251.42, 180.0, 10.0, 180.0]
    drt_movel(pos_l, vel_l, acc_l, 6,radius,ref,mode,blendType,syncType)

    dir_fd = [1,0,1,0,0,0]; fd = [50.0, 0.0, 20.0,0,0,0]
    drt_set_desired_force(fd,dir_fd,ref,6.0,mode)

    dir_fd = [1,1,0,0,0,0]; fd = [50.0, -20.0, 0.0,0,0,0]
    drt_set_desired_force(fd,dir_fd,ref,6.0,mode)

    dir_fd = [1,0,0,0,0,0]; fd = [50.0, 0.0, 0.0,0,0,0]
    drt_set_desired_force(fd,dir_fd,ref,0.0,mode)

    pos_l = [723.5, -10.22, 251.42, 180.0, 10.0, 180.0]
    drt_movel(pos_l, vel_l, acc_l, 15,radius,ref,mode,blendType,syncType)

    # Unplug
    drt_release_cmplt(); drt_release_force();
    pos_l = [603.5, -10.22, 251.42, 180.0, 10.0, 180.0]
    drt_movel(pos_l, vel_l, acc_l, time_l,radius,ref,mode,blendType,syncType)

