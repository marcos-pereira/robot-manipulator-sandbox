from dqrobotics import *
import numpy as np
from dqrobotics.utils import *
pinv = DQ_LinearAlgebra.pinv
from dqrobotics.robots import KukaLw4Robot


class PseudoinverseController:

    ## Robot kinematic model
    robot_ = KukaLw4Robot.kinematics()

    ## Controller gain
    gain_ = 0

    ## Last error signal
    last_error_signal_ = DQ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    ## Last control signal
    last_control_signal_ = vec8(DQ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

    ## PseudoinverseController constructor
    # @param robot is the robot kinematics model
    # @param gain is the controller gain
    def __init__(self, robot, gain):
        self.robot_ = robot
        self.gain_ = gain

    ## Compute control signal
    ## The control signal are joint velocities
    # @param q are the instantaneous measured joint positions
    # @param task_reference is the task reference, i.e., the desired pose
    def compute_control_signal(self, q, task_reference):
        task_variable = self.robot_.fkm(q)
        task_error = vec8(task_variable-task_reference)
        J = self.robot_.pose_jacobian(q)

        u = -self.gain_ * np.matmul(pinv(J), task_error)

        self.last_error_signal_ = task_error
        self.last_control_signal_ = u

        return u

    ## Get controller last error signal
    def get_last_error_signal(self):
        return self.last_error_signal_

    ## Get controller last control signal
    def get_last_control_signal(self):
        return self.last_control_signal_




