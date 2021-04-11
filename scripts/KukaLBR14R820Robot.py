# coding=utf-8
"""
(C) Copyright 2015 DQ MACRO Developers
This file is part of KukaLBR4p
    KukaLBR14R820 is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    KukaLBR14R820 is distributed in the hope that it will be useful,
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
class KukaLBR14R820Robot():
    @staticmethod
    def kinematics():
        # Parameters by hand
        kuka_lbr14_theta = np.array([pi,    pi,     pi,   pi,   pi,    pi,  0])
        kuka_lbr14_d = np.array([0.2075, 0,     0.42, 0,   0.4054, 0,   0.129])
        kuka_lbr14_a = np.array([0,     0,     0,   0,   0,    0,   0])
        kuka_lbr14_alpha = np.array([pi2,   pi2, -pi2, -pi2, pi2, pi2, 0])
        kuka_lbr14_type = np.array([0, 0, 0, 0, 0, 0, 0])

        # Parameters from CoppelliaSim DH extractor
        # Do not work well
        # kuka_lbr14_theta = np.array([-pi, -pi, 0, -pi, -pi/2, -pi, 0])
        # kuka_lbr14_d = np.array([0.2075, 0, 0.42, 0, 0.40, 0, 0.129])
        # kuka_lbr14_a = np.array([0, 0, 0, 0, 0.0438, 0.0438, 0])
        # kuka_lbr14_alpha = np.array([pi2, pi2, pi2, pi2, -pi2, -pi2, 0])
        # kuka_lbr14_type = np.array([0, 0, 0, 0, 0, 0, 0])

        kuka_DH_matrix = np.array([kuka_lbr14_theta,
                                   kuka_lbr14_d,
                                   kuka_lbr14_a,
                                   kuka_lbr14_alpha,
                                   kuka_lbr14_type])
        kuka_lbr14 = DQ_SerialManipulatorDH(kuka_DH_matrix, 'standard')
        return kuka_lbr14