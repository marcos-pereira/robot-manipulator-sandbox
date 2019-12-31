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
from interface import Interface


def control():
	print("control()")

	## Defining robot kinematic model
	robot = KukaLw4Robot.kinematics()

	## Desired pose
	translation = DQ([0.0, 0.1, 0.1, 0.1])
	rotation = DQ([1.0, 0.0, 0.0, 0.0])
	xd = rotation + E_ * 0.5 * translation * rotation
	# xd = robot.fkm((0,math.pi,0,math.pi,0,math.pi/2,0))
	# xd = robot.fkm((0.0, -math.pi/6, 0.0, math.pi/2.0, math.pi/6, 0.0, 0.0))

	## Defining initial target joints
	theta_init = np.array([0.0, -math.pi/3.7, 0.0, math.pi/2.0, math.pi/3.7, 0.0, 0.0])

	## Controller gain
	gain = 1

	## Defining controller
	pseudoinverse_controller = PseudoinverseController(robot, gain)

	## Defining communication interface
	interface = Interface(robot.get_dim_configuration_space())

	## Initialize node
	rospy.init_node('robot_interface', anonymous=True)

	## Control loop sampling time (seconds)
	sampling_time = 0.05

	## ROS loop frequency rate
	rate = rospy.Rate(1/sampling_time)

	## State machine states
	k_set_init_config = 1
	k_run_control = 2
	k_end_state = 3

	## Count initial configuration iterations
	set_init_config_counter = 1
	init_config_iterations = 100

	robot_state = k_set_init_config

	while not rospy.is_shutdown():

		if robot_state == k_set_init_config:

			print("k_set_init_config")

			## Set robot init config
			interface.send_joint_position(theta_init)

			if set_init_config_counter > init_config_iterations:

				## Reset counter
				set_init_config_counter = 0

				## Init pose
				x_init = robot.fkm(interface.get_joint_positions())

				## Set next state
				robot_state = k_run_control

			set_init_config_counter += 1

		if robot_state == k_run_control:

			print("k_run_control")

			## Get joint positions from robot
			joint_positions = interface.get_joint_positions()

			## Desired pose to be set
			x_set = xd*x_init

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
			if np.linalg.norm(task_error) < 0.001:
				robot_state = k_end_state

		if robot_state == k_end_state:
			print("k_end_state")

		rate.sleep()

def main():
	control()

if __name__ == '__main__':

    main()

    
