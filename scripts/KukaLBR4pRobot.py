# coding=utf-8
"""
(C) Copyright 2015 DQ MACRO Developers
This file is part of KukaLBR4p
    KukaLBR4p is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    KukaLBR4p is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
    You should have received a copy of the GNU Lesser General Public License
    along with Little_John.  If not, see <http://www.gnu.org/licenses/>.
Contributors:
- Marcos da Silva Pereira  (marcos.si.pereira@gmail.com)
Version History:
-- Initial Version
"""
from dqrobotics import *
from dqrobotics.robot_modeling import DQ_SerialManipulatorDH
import numpy as np
from math import pi
pi2 = pi/2


class KukaLBR4pRobot:

    @staticmethod
    def kinematics():
        kuka_lbr4p_theta = np.array([0,     0,     0,   0,   0,    0,   0])
        kuka_lbr4p_d = np.array([0.2,     0,     0.4, 0,   0.39, 0,   0.078])
        kuka_lbr4p_a = np.array([0,     0,     0,   0,   0,    0,   0])
        kuka_lbr4p_alpha = np.array([pi2,   -pi2,  pi2, -pi2, pi2, -pi2, 0])
        kuka_lbr4p_type = np.array([0, 0, 0, 0, 0, 0, 0])
        kuka_DH_matrix = np.array([kuka_lbr4p_theta,
                                   kuka_lbr4p_d,
                                   kuka_lbr4p_a,
                                   kuka_lbr4p_alpha,
                                   kuka_lbr4p_type])
        kuka_lbr4p = DQ_SerialManipulatorDH(kuka_DH_matrix, 'standard')

        return kuka_lbr4p
