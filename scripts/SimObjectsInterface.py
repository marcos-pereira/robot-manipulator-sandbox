import rospy
import numpy as np

from geometry_msgs.msg import Pose

from dqrobotics import *

class SimObjectsInterface:

    ## Interface constructor
    # @param node_name is the node name
    def __init__(self, node_name):

        # Store object pose
        self.object_pose_ = np.zeros(7)

        ## Object pose contains object position (x,y,z) and quaternion orientation(x, y, z, w)
        # self.object_pose_.resize(7)

        ## Make w always 1 to enable always calculating dq pose
        self.object_pose_[6] = 1

        self.subscriber_object_ = rospy.Subscriber(node_name, Pose, self.object_pose_callback)

    ## ROS callback method for object pose
    # @param data is the Pose ROS message
    def object_pose_callback(self, data):
        self.object_pose_[0] = data.position.x
        self.object_pose_[1] = data.position.y
        self.object_pose_[2] = data.position.z
        self.object_pose_[3] = data.orientation.x
        self.object_pose_[4] = data.orientation.y
        self.object_pose_[5] = data.orientation.z
        self.object_pose_[6] = data.orientation.w

    ## Get object pose vector
    def object_pose(self):
        return self.object_pose_

    ## Get object dual quaternion (dq) pose
    def dq_object_pose(self):
        translation = self.object_pose_[0]*i_ + self.object_pose_[1]*j_ + self.object_pose_[2]*k_
        rotation = DQ([self.object_pose_[6], self.object_pose_[3], self.object_pose_[4], self.object_pose_[5]])
        dq_object_pose = rotation + E_*0.5*translation*rotation
        dq_object_pose = normalize(dq_object_pose)
        return dq_object_pose
