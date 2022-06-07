#!/usr/bin/env python3
import rospy
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

control_type = "torque" #"gravity" #"velocity" # 

traj = Trajectory(6)
traj_flag = [0]*6
traj_res = [0]*6
motion = 1


rate=200
# works with the default example only because I'm not dragging in their libraries to try and parameterize this.

jscb_count=0
rob_pos=[]

def jointstatecallback(msg: JointState):
    global jscb_count, rob_pos
    jscb_count=jscb_count+1
    rob_pos=list(msg.position)

# ROS boilerplate
print("Setting up node roscontrol_test, I will wiggle the second last joint ~10deg each way.")
rospy.init_node('roscontrol_test')

drt_servo_on=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/system/set_robot_control', SetRobotControl)
retval=drt_servo_on(robot_control = 3)
print(retval)

drt_set_mode=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/system/set_robot_mode', SetRobotMode)
retval=drt_set_mode(robot_mode = ROBOT_MODE_AUTONOMOUS)
print(retval)

drt_set_sys=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/system/set_robot_system', SetRobotSystem)
retval=drt_set_sys(robot_system = ROBOT_SYSTEM_REAL)
print(retval)


drt_connect=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/connect_rt_control', ConnectRTControl)
retval=drt_connect(ip_address =  "192.168.12.249", port = 12347)
#retval=drt_connect(ip_address =  "192.168.0.114", port = 12347)
if not retval:
   raise SystemExit('realtime connect failed')
print("connected")


drt_setout=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/set_rt_control_output', SetRTControlOutput)
retval=drt_setout(version = "v1.0", period = 0.001, loss = 10)

if not retval:
   raise SystemExit('realtime set output failed')
print("settup")
# start_rt_control!!
drt_start=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/start_rt_control', StartRTControl)
retval=drt_start()
if not retval:
   raise SystemExit('realtime start control failed')
print("strated")

drt_read=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/read_data_rt', ReadDataRT)
drt_write=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/write_data_rt', WriteDataRT)

print("setup has done")

if control_type=="velocity":
    command = rospy.Publisher("/dsr01m0609/speedj_rt_stream", SpeedJRTStream, queue_size=1)
elif control_type=="torque" or  control_type == "gravity":
    command = rospy.Publisher("/dsr01m0609/torque_rt_stream", TorqueRTStream, queue_size=1)
governor=rospy.Rate(rate)

drt_safety=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/system/set_robot_safety_mode', SetRobotSafetyMode)
print(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE)
retval=drt_safety(robot_safety_mode = SAFETY_MODE_AUTONOMOUS, robot_safety_mode_event=SAFETY_MODE_EVENT_MOVE)
print(retval)

if control_type=="velocity":
    cmd_vel=SpeedJStream()
    cmd_vel.time=1/rate
elif control_type == "torque" or control_type == "gravity" :
    cmd_tor=TorqueRTStream()
    cmd_tor.tor = [0]*6
    cmd_tor.time = 1/rate
    
init_time=time.time()
qd=[0]*6
qd_dot=[0]*6
tor_ext_tmp=[0]*6
buf_tor_ext=[0]*6
tor_ext=[0]*6
frq_cut_off=10
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
    
    if motion==1 and traj_flag[0]==0:
        trj_q=[0,0,90,0,90,0]
        motion+=1
        traj_flag=[1]*6
    elif motion==2 and traj_flag[0]==0:
        trj_q=[0,0,0,0,0,0]
        motion+=1
        traj_flag=[1]*6
    elif motion==3 and traj_flag[0]==0:
        trj_q=[10, -10, 20, -30, 10, 20]
        motion+=1
        traj_flag=[1]*6    
    elif motion==4 and traj_flag[0]==0:
        trj_q=[25, 0, 10, -50, 20, 40]
        motion+=1
        traj_flag=[1]*6  
    elif motion==5 and traj_flag[0]==0:
        trj_q=[50, 50, 50, 50, 50, 50]
        motion+=1
        traj_flag=[1]*6      
    elif motion==6 and traj_flag[0]==0:
        trj_q=[30, 10, 30, -20, 10, 60]
        motion+=1
        traj_flag=[1]*6     
    elif motion==7 and traj_flag[0]==0:
        trj_q=[20, 20, 40, 20, 0, 90]
        motion=0
        traj_flag=[1]*6    
        
        
    for i in range(6):
        if traj_flag[i]==1:
            traj.SetPolynomial5th(i,q[i],trj_q[i],g_time,2.0)
            qd[i]=q[i]
            traj_flag[i]=2
        
        elif traj_flag[i]==2:
            tmp_res,tmp_flag=traj.Polynomial5th(i,g_time)
            qd[i] = tmp_res[0]
            qd_dot[i] = tmp_res[1]
            if tmp_flag == 0:
                traj_flag[i]=0
    
    
    if control_type == "velocity":
        # generate a signal for the robot to track
        cmd_vel.vel[4]=10*sin(2*3.141592*time)
        cmd_vel.acc[4]=10*2*3.141592*cos(2*3.141592*time)
        
        #publish command
        command.publish(cmd_vel)
    
        #print(cmd_pos.vel)
        #print(cmd_pos.acc)
        #print("")
        
    elif control_type == "torque":
        
        
        for i in range(6):
            tor_ext[i] = alpha*tor_ext_tmp[i] + (1-alpha)*buf_tor_ext[i];
            if i == 0:
                cmd_tor.tor[i] = 10*(qd[i]-q[i])+0.1*(qd_dot[i]-q_dot[i])+tor_g[i]+2.5*tor_ext[i]
            elif i == 1:
                cmd_tor.tor[i] = 10*(qd[i]-q[i])+0.1*(qd_dot[i]-q_dot[i])+tor_g[i]+2*tor_ext[i]
            elif i == 2:
                cmd_tor.tor[i] = 10*(qd[i]-q[i])+0.1*(qd_dot[i]-q_dot[i])+tor_g[i]+2.5*tor_ext[i]
            elif i == 3:
                cmd_tor.tor[i] = 10*(qd[i]-q[i])+0.1*(qd_dot[i]-q_dot[i])+tor_g[i]+3*tor_ext[i]
            elif i == 4:
                cmd_tor.tor[i] = 10*(qd[i]-q[i])+0.1*(qd_dot[i]-q_dot[i])+tor_g[i]+4*tor_ext[i]
            elif i == 5:
                cmd_tor.tor[i] = 10*(qd[i]-q[i])+0.1*(qd_dot[i]-q_dot[i])+tor_g[i]+6*tor_ext[i]
            buf_tor_ext[i]=tor_ext[i]
        command.publish(cmd_tor)

        print(cmd_tor.tor)
        
    elif control_type == "gravity":
        for i in range(6):
            tor_ext[i] = alpha*tor_ext_tmp[i] + (1-alpha)*buf_tor_ext[i];
            cmd_tor.tor[i] = tor_g[i]+2.5*tor_ext[i]
            buf_tor_ext[i]=tor_ext[i]
        command.publish(cmd_tor)

        print(cmd_tor.tor)
    # 10Hz
    governor.sleep()
    

