#!/usr/bin/env python

from dqrobotics import *
from dqrobotics.robots import KukaLw4Robot
import numpy as np
from dqrobotics.utils import *
import math
import time
pinv = DQ_LinearAlgebra.pinv
import rospy
from controller import PseudoinverseController
from dqrobotics.robot_control import DQ_PseudoinverseController, ControlObjective
from interface import Interface
from SimObjectsInterface import SimObjectsInterface
from KukaLBR14R820Robot import KukaLBR14R820Robot
from KukaLBR4pRobot import KukaLBR4pRobot


def control():
	print("control()")

	## Robot types
	robot_LBR14R820 = 1
	robot_LBR4p = 0

	## Choose robot type
	robot_type = robot_LBR14R820

	## Defining robot kinematic model
	if robot_type == robot_LBR14R820:
		robot = KukaLBR14R820Robot.kinematics()
	if robot_type == robot_LBR4p:
		robot = KukaLBR4pRobot.kinematics()

	## Set robot base transformation
	# Translation between robot frame and joint1 frame
	if robot_type == robot_LBR14R820:
		base_p = -0.0073 * i_ + 0.0001 * j_ + 0.0739 * k_
	if robot_type == robot_LBR4p:
		base_p = -0.00008 * i_ + 0.0001*j_ + 0.1010 * k_

	x_base = 1.0 + E_ * 0.5 * base_p
	robot.set_base_frame(x_base)
	robot.set_reference_frame(x_base)

	## Set robot effector transformation
	# Distance between LBR connection and gripper attachpoint
	effector_p = 0.1190 * k_
	x_effector = 1.0 + E_ * 0.5 * effector_p
	robot.set_effector(x_effector)

	## Desired pose example
	translation = DQ([0.0, 0.1, 0.1, 0.1])
	rotation = DQ([1.0, 0.0, 0.0, 0.0])
	xd = rotation + E_ * 0.5 * translation * rotation
	# xd = robot.fkm((0,math.pi,0,math.pi,0,math.pi/2,0))
	# xd = robot.fkm((0.0, -math.pi/6, 0.0, math.pi/2.0, math.pi/6, 0.0, 0.0))

	## Create linear trajectory
	x_trajectory = np.arange(0, 0.11, 0.01)
	y_trajectory = np.arange(0, 0.11, 0.01)
	z_trajectory = np.arange(0, 0.11, 0.01)

	## Create circular trajectory
	radius = 0.1
	x_trajectory = np.arange(0, 2*math.pi, 0.01)
	y_trajectory = np.arange(0, 2*math.pi, 0.01)
	z_trajectory = np.zeros(len(x_trajectory))

	## Create pose trajectory
	translation_trajectory = []
	for ii in range(len(x_trajectory)):
		## Linear trajectory
		translation = DQ([0.0, x_trajectory[ii], y_trajectory[ii], z_trajectory[ii]])

		## Circular trajectory
		translation = DQ([0.0, radius*math.sin(x_trajectory[ii]), radius*math.cos(y_trajectory[ii]), z_trajectory[ii]])

		rotation = DQ([1.0, 0.0, 0.0, 0.0])
		xd_trajectory = rotation + E_ * 0.5 * translation * rotation
		translation_trajectory.append(xd_trajectory)

	## Defining initial target joints
	theta_init = np.array([0.0, -math.pi/3.7, 0.0, math.pi/2.0, math.pi/3.7, 0.0, 0.0])

	## Control parameters
	gain = 1.0
	desired_error_norm = 0.001

	## Defining controller
	pseudoinverse_controller = PseudoinverseController(robot, gain)
	# DQ robotics DQ_PseudoinverseController:
	# pseudoinverse_controller = DQ_PseudoinverseController(robot)
	# pseudoinverse_controller.set_control_objective(ControlObjective.Pose)
	# pseudoinverse_controller.set_gain(gain)
	# pseudoinverse_controller.set_damping(0.001)

	## Defining communication interface
	interface = Interface(robot.get_dim_configuration_space())

	## Interface to get pose of objects
	if robot_type == robot_LBR14R820:
		robot_object = SimObjectsInterface('/LBR_iiwa_14_R820')
	if robot_type == robot_LBR4p:
		robot_object = SimObjectsInterface('/LBR4p')
	place_frame0 = SimObjectsInterface('/PlaceFrame0')

	## Initialize node
	rospy.init_node('robot_interface', anonymous=True)

	## Control loop sampling time (seconds)
	sampling_time = 0.005

	## ROS loop frequency rate
	rate = rospy.Rate(1/sampling_time)

	## State machine states
	k_set_init_config = 1
	k_run_pose_control = 2
	k_end_state = 3
	k_run_trajectory_control = 4

	## Count initial configuration iterations
	set_init_config_counter = 1
	init_config_iterations = 500

	## Count trajectory steps
	trajectory_counter = 0

	robot_state = k_set_init_config

	while not rospy.is_shutdown():

		if robot_state == k_set_init_config:

			print("k_set_init_config")

			## Set robot init config
			interface.send_joint_position(theta_init)

			## Command gripper to close
			interface.send_gripper_command(1)

			if set_init_config_counter > init_config_iterations:

				## Reset counter
				set_init_config_counter = 0

				## Command gripper to open
				interface.send_gripper_command(0)

				## Init pose
				x_init = robot.fkm(interface.get_joint_positions())

				## Set next state
				robot_state = k_run_pose_control

			set_init_config_counter += 1

		if robot_state == k_run_pose_control:

			print("k_run_pose_control")

			## Get joint positions from robot
			joint_positions = interface.get_joint_positions()

			## Effector initial pose
			effector_p = 0.0 * k_
			effector_t = 1 + E_ * 0.5 * effector_p
			effector_phi = math.pi
			effector_n = -j_
			effector_r = math.cos(effector_phi / 2.0) + effector_n * math.sin(effector_phi / 2.0)
			effector_pose = effector_t*effector_r

			## Desired pose to be set
			x_set = conj(robot_object.dq_object_pose())*place_frame0.dq_object_pose()*effector_pose

			## Compute control signal
			## The control signal is the robot joint velocities
			u = pseudoinverse_controller.compute_control_signal(joint_positions, x_set)
			# If using DQ_PseudoinverseController:
			# u = pseudoinverse_controller.compute_setpoint_control_signal(joint_positions, vec8(x_set))

			## We are actuating in the robot with joint position commands
			## So we integrate the joint velocities
			theta_set = joint_positions + u * sampling_time

			## Send joint positions to robot
			interface.send_joint_position(theta_set)

			## Get controller error
			task_error = pseudoinverse_controller.get_last_error_signal()
			print("task_error ", np.linalg.norm(task_error))

			## Verify if desired error was reached
			if np.linalg.norm(task_error) < desired_error_norm:
				robot_state = k_end_state

		if robot_state == k_run_trajectory_control:

			print("k_run_trajectory_control")

			## Get joint positions from robot
			joint_positions = interface.get_joint_positions()

			## Desired pose to be set
			x_set = translation_trajectory[trajectory_counter]*x_init

			## Compute control signal
			## The control signal is the robot joint velocities
			u = pseudoinverse_controller.compute_control_signal(joint_positions, x_set)

			## We are actuating in the robot with joint position commands
			## So we integrate the joint velocities
			theta_set = joint_positions + u * sampling_time

			## Send joint positions to robot
			interface.send_joint_position(theta_set)

			## Get controller error
			task_error = pseudoinverse_controller.get_last_error_signal()
			print("task_error ", np.linalg.norm(task_error))

			## Verify if desired error was reached
			if np.linalg.norm(task_error) < 0.01:
				trajectory_counter += 1

			## If trajectory has finished
			if trajectory_counter == len(translation_trajectory):
				robot_state = k_end_state

		if robot_state == k_end_state:
			print("k_end_state")

		rate.sleep()

def main():
	control()

if __name__ == '__main__':

    main()

    
