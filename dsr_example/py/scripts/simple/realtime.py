#!/usr/bin/env python3
import rospy
import os
import threading, time
import sys
sys.dont_write_bytecode = True
#sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m0609"
from DSR_ROBOT import *
__dsr__id = ROBOT_ID
__dsr__model = ROBOT_MODEL

rospy.init_node('roscontrol_test')
rate=100
governor=rospy.Rate(rate)

# connect_rt_control    
#  ip_address =  192.168.137.100
#  port = 12345
drt_connect=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/connect_rt_control', ConnectRTControl)
retval=drt_connect(ip_address =  "192.168.12.249")
#retval=drt_connect(ip_address =  "192.168.0.114")
#retval=drt_connect(ip_address =  "127.0.0.1")
if not retval:
   raise SystemExit('realtime connect failed')
print("connected")

retval=set_robot_control(CONTROL_SERVO_ON)
print(CONTROL_SERVO_ON)
print(retval)
   
retval=set_robot_mode(ROBOT_MODE_AUTONOMOUS)
print(retval)
retval=set_robot_system(ROBOT_SYSTEM_REAL)
print(retval)

# set_rt_control_output
#  version = "v1.0"
#  period = 0.001    //sampling time, here 100Hz
#  loss = 10     //unknown, currently unused by doosan firmware.
drt_setout=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/set_rt_control_output', SetRTControlOutput)
retval=drt_setout( version = "v1.0", period = 0.001, loss = 10)
if not retval:
   raise SystemExit('realtime set output failed')
print("settup")
# start_rt_control!!
drt_start=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/start_rt_control', StartRTControl)
retval=drt_start()
if not retval:
   raise SystemExit('realtime start control failed')
print("strated")
# set up read,  write, stop and disconnect proxies
drt_read=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/read_data_rt', ReadDataRT)
drt_write=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/write_data_rt', WriteDataRT)
readdata=RobotStateRT()

print("setup has done")

# -------------main loop ------------------
while not rospy.is_shutdown():
   # read_data_rt  //all the data you could ever want, and some you definitly dont
   
   readdata=drt_read()
   #print(readdata)
   ## write_data_rt //only force mode it seems? we're missing a servoj_rt or speedj_rt service call. leave this disabled.
   retval=drt_write( external_force_torque=[0.0,0.0,0.0,0.0,0.0,10.0], 
                     external_digital_input=0,
                     external_digital_output=0,
                     external_analog_input=[0.0,0.0,0.0,0.0,0.0,0.0],
                     external_analog_output=[0.0,0.0,0.0,0.0,0.0,0.0])

# ----------------CLEANUP-------------

# stop_rt_control
drt_stop=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/stop_rt_control', StopRTControl)
retval = drt_stop()
# no if, because when it fails... well there's not much i can do to stop it now is there?
print("Stop returns: " + str(retval))

# disconnect_rt_control_cb
drt_drop=rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/realtime/disconnect_rt_control', DisconnectRTControl)
retval = drt_drop()
print("Disconnect returns: " + str(retval))
