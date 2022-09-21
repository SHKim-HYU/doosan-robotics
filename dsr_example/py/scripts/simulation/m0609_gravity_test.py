#!/usr/bin/env python3
## OCP for point-to-point motion and visualization of a KUKA robot arm

from tasho import task_prototype_rockit as tp
from tasho import input_resolution, world_simulator
from tasho import robot as rob
import casadi as cs
import numpy as np
import matplotlib.pyplot as plt
from tasho.utils import dist_computation
import pybullet as p
import time
import os
import math
import rospy

from multiprocessing import Process, Manager
import tf
import modern_robotics as mr

manager = Manager()
_q = manager.dict()

_q['q_r'] = [0.0]*6; _q['q_r_dot'] = [0.0]*6; _q['trq_r_g'] = [0.0]*6; 
_q['q_n'] = [0.0]*6; _q['q_n_dot'] = [0.0]*6; _q['trq_n_g'] = [0.0]*6; 


def nominal_run():
	print("Task specification and visualization of P2P OCP")
	rospy.init_node('pybullet_control_test')


	script_dir=os.path.dirname(__file__)
	horizon_size = 20
	t_ocp = 0.2
	max_joint_acc = 120*3.14159/180
	max_joint_vel = 120*3.14159/180

	#Different OCP options
	time_optimal = False
	coll_avoid = False
	frame_constraint = False
	robot_choice = 'm0609' #'kinova'#'iiwa7' #
	ocp_control =  'torque_resolved' #'acceleration_resolved' #
	L1_pathcost = False #Heuristically time optimal solution

	robot = rob.Robot(robot_choice)

	jac_fun = robot.set_kinematic_jacobian(name="jac_fun",q = 6)
	robot.set_joint_acceleration_limits(lb = -max_joint_acc, ub = max_joint_acc)
	robot.set_joint_velocity_limits(lb = -max_joint_vel, ub = max_joint_vel)

	if time_optimal:
		tc = tp.task_context()
		tc.minimize_time(100)
	else:
		tc = tp.task_context(t_ocp*horizon_size)

	if ocp_control == 'acceleration_resolved':
		q, q_dot, q_ddot, q0, q_dot0 = input_resolution.acceleration_resolved(tc, robot, {})
	elif ocp_control == 'torque_resolved':
		q, q_dot, q_ddot, tau, q0, q_dot0 = input_resolution.torque_resolved(tc, robot, {'forward_dynamics_constraints': False})

	fk_vals = robot.fk(q)[6]
	print(fk_vals)
	T_goal = np.array([[0, 1, 0, 0.6], [-1, 0, 0, 0.2], [0, 0, 1, 2.0], [0, 0, 0, 1]])

	#q0_val = [-1.23284074, -0.35897528,  1.94483444,  1.5709, 1.5709,
    #     1.50381687, -0.91972968,  1.54674921,  0.33006886]
	# q0_val = [-0.75202906,  0.17875017,  0.64365555,  2.33571781, -0.23352359,  0.97565258,
 	# -0.59070319,  1.68163271, -0.17681645]
	q0_val = [0, 0, 0.7, 0, 1.5709, 0]


	obj = world_simulator.world_simulator(plane_spawn = True, bullet_gui = True)
	obj.visualization_realtime = True
	obj.torque_control_mode = True
	position = [0.0, 0.0, 0.0]
	orientation = [0.0, 0.0, 0.0, 1.0]
	dsrID = obj.add_robot(position, orientation, robot_choice)

	joint_indices = [1, 2, 3, 4, 5, 6]
	obj.resetJointState(dsrID, joint_indices, q0_val)
	print(obj.getJointInfoArray(dsrID))


	e = [0]*len(joint_indices)
	e_dot = [0]*len(joint_indices)
	tau = [0]*len(joint_indices)

	pGain = [200,	500,	500,	50,		50,		50]
	dGain = [2.5,	25,		25,		0.5,	0.5,	0.5]

	toc_cum=0;
	toc_buf=0;
	i=0
	init_time = time.time()
	while True:
		start_time = time.time()
		#qd = [0, 0, 0, 0, 0, 0, 0, 0, math.sin(2*math.pi*0.001*i)]
		# qd = [-1.23284074, -0.35897528,  1.94483444,  1.47173868, -0.44895069,
        #  1.50381687, -0.91972968,  1.54674921, math.sin(2*math.pi*0.001*i)]
		# qd_dot = [0, 0, 0, 0, 0, 0, 0, 0, (2*math.pi*0.001)*math.cos(2*math.pi*0.001*i)]

		# qd = [1, 1, math.pi*math.sin(2*math.pi*0.05*i), 0, 0, 0, 0, 0, 0]
		# qd_dot = [0, 0, math.pi*(2*math.pi*0.05)*math.cos(2*math.pi*0.05*i), 0, 0, 0, 0, 0, 0]
		qd = [0, 0, 0.7, 0, 1.5709, 0]
		qd_dot = [0, 0, 0, 0, 0, 0]

		q_act = obj.readJointState(dsrID,joint_indices)

		q_arr=[q_act[0][0],q_act[1][0],q_act[2][0],q_act[3][0],q_act[4][0],q_act[5][0]]
		q_dot_arr=[q_act[0][1],q_act[1][1],q_act[2][1],q_act[3][1],q_act[4][1],q_act[5][1]]
		q_vec=cs.vertcat(q_act[0][0],q_act[1][0],q_act[2][0],q_act[3][0],q_act[4][0],q_act[5][0])
		q_dot_vec=cs.vertcat(q_act[0][1],q_act[1][1],q_act[2][1],q_act[3][1],q_act[4][1],q_act[5][1])
		#print(q_arr)
		#print(q_dot_arr)

		jac, jac_rot=jac_fun(q_arr)
		J_geo = cs.vertcat(jac_rot,jac)
		G_vals=robot.G(q_arr).full().reshape(len(joint_indices))
		twist_s=J_geo@q_dot_vec
		#print("twist_s",twist_s)
		#print(G_vals)

		for j in range(len(joint_indices)):
			e[j] = qd[j] - q_act[j][0]
			e_dot[j] = qd_dot[j] - q_act[j][1]
			# tau[j] = pGain[j]*e[j] + dGain[j]*e_dot[j] + G_vals[j]
			tau[j] = G_vals[j]
			#tau[j] = 0

		#print(G_vals)
		#print(tau)

		# Compute
		fk_act_vals=robot.fk(q_arr)[6][0:3,3]
		print(fk_act_vals)
		T_fk=cs.vertcat(robot.fk(q_arr)[6][0:3,3])

		tic=time.time()
		fd_vals=robot.fd(q_arr,q_dot_arr,tau)
		toc=time.time()-tic
		toc_cum=toc_cum+toc
		if toc_buf<toc:
			toc_buf=toc
		#print(fd_vals)

		# Arm
		obj.setController(dsrID, "torque", joint_indices, targetTorques = tau)

		end_time = time.time()
		loop_time = (end_time-start_time)*1000 # [ms]
		global_time = end_time-init_time
		print("################ Arm joint state ################")
		print("Joint 1: qd1=%f,\tqd1_dot=%f\n\tq1=%f,\tq1_dot=%f,\n\ttorque_des=%f" %(qd[0],qd_dot[0],q_act[0][0],q_act[0][1],tau[0]))
		print("Joint 2: qd2=%f,\tqd2_dot=%f\n\tq2=%f,\tq2_dot=%f,\n\ttorque_des=%f" %(qd[1],qd_dot[1],q_act[1][0],q_act[1][1],tau[1]))
		print("Joint 3: qd3=%f,\tqd3_dot=%f\n\tq3=%f,\tq3_dot=%f,\n\ttorque_des=%f" %(qd[2],qd_dot[2],q_act[2][0],q_act[2][1],tau[2]))
		print("Joint 4: qd4=%f,\tqd4_dot=%f\n\tq4=%f,\tq4_dot=%f,\n\ttorque_des=%f" %(qd[3],qd_dot[3],q_act[3][0],q_act[3][1],tau[3]))
		print("Joint 5: qd5=%f,\tqd5_dot=%f\n\tq5=%f,\tq5_dot=%f,\n\ttorque_des=%f" %(qd[4],qd_dot[4],q_act[4][0],q_act[4][1],tau[4]))
		print("Joint 6: qd6=%f,\tqd6_dot=%f\n\tq6=%f,\tq6_dot=%f,\n\ttorque_des=%f" %(qd[5],qd_dot[5],q_act[5][0],q_act[5][1],tau[5]))
		print("################################################")
		print(cs.sqrt(cs.det(J_geo@J_geo.T)))
		print()
		#print(fk_act_vals[0:3,3])
		print("\n\n")
		#"""
		obj.run_simulation(1)
		#print(fk_vals.type)
		i+=1


if __name__ == '__main__':

	nominal_task = Process(target=nominal_run, args=())

	try:
		nominal_task.start()
		nominal_task.join()

	except KeyboardInterrupt:
		nominal_task.terminate()