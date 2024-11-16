#!/usr/bin/env python3

import numpy as np

from skrobot.coordinates.math import random_quaternion
from skrobot.coordinates.math import random_translation
from skrobot.coordinates.math import quaternion2rpy
from skrobot.model import RobotModel
from skrobot.model import RotationalJoint
from skrobot.model import Cylinder
from skrobot.coordinates import Coordinates
from skrobot.viewers import TrimeshSceneViewer

import rospy
from geometry_msgs.msg import PoseStamped

from urdf_generator import AppleKinovo, SymbolicParameter, SymbolicParameterList

rospy.init_node("ik_interface")

while True:
    try:
        if rospy.has_param("/robot_configuration"):
            break
    except KeyboardInterrupt:
       raise KeyboardInterrupt
    

linkinfo = rospy.get_param("/robot_configuration")

symparam_list = SymbolicParameterList([
    SymbolicParameter("link1", "link2", 
                      np.array(linkinfo["imua0201c__to__imu6ca3b8_pos"]), 
                      np.array(linkinfo["imua0201c__to__imu6ca3b8_rpy"])),
    SymbolicParameter("link2", "link3",
                      np.array(linkinfo["imua00fc8__to__imua015d0_pos"]), 
                      np.array(linkinfo["imua00fc8__to__imua015d0_rpy"])),
])

urdf_str = AppleKinovo(symparam_list).make_urdf()

print(urdf_str)

out_urdfpath = '/tmp/robot.urdf'
with open(out_urdfpath, 'w') as f:
    f.write(urdf_str)

robot_model = RobotModel()
robot_model.load_urdf_file(open(out_urdfpath))


###### ARMH7 ##########

rospy.logwarn("initialize RCB4")

import numpy as np
from rcb4.armh7interface import ARMH7Interface
import time

# class GripperInterface(ARMH7Interface):
#     def auto_open2(self):
#         for _ in range(3):
#             try:
#                 return self.interface.auto_open()
#             except Exception as e:
#                 print(e)
#         # return self.interface.auto_open()
    
#     def gripper_open(self):
#         return self.interface.angle_vector([70],[12])
    
#     def _gripper_close_adaptive(self, target_av, steps=10, gripper_id=12):
#         interface = self.interface
#         av_idx = interface.servo_id_to_index(gripper_id)
#         av = interface.angle_vector()[av_idx]
        
#         av_steps = np.linspace(av, target_av, steps)[1:]
#         dav = av_steps[1] - av_steps[0]

#         for interp_av in av_steps:
#             self.interface.angle_vector([interp_av],[gripper_id])
#             print("sent", interp_av)
#             time.sleep(1)
#             _av = interface.angle_vector()[av_idx]
#             print("result", _av)
#             print(av, _av)
#             if np.abs(av - _av) / np.abs(dav) < 0.5:
#                 print("breaked")
#                 break
#             # print("av - _av", av - _av)
#             # print("dav", dav)
#             av = _av
    
#     def gripper_close(self):
#         return self._gripper_close_adaptive(-105)
    
#     def _gripper_close(self):
#         return self.interface.angle_vector([-105],[12])

# if __name__ == "__main__":
interface = ARMH7Interface()
print(interface.auto_open())
interface.hold()

## initialize
# av = (1 - 2*np.random.rand(6)) * np.pi * 0.67

rospy.logwarn(f"servo ids = {interface.search_servo_ids()}")

AXIS_DIRECTION = np.array([1, 1, 1, 1, 1])


import time

class RobotController:

    def __init__(self):
        self.elapsed_time = 0
        self.toc = None
        self.prev_stamp = -1.

    def update_endpos(self, msg):
        if self.toc is None:
            self.toc = time.time()
        tic = time.time()

        elapsed_time = self.toc - tic
        if self.prev_stamp + elapsed_time > msg.header.stamp.to_sec():
            return None

        self.prev_stamp = msg.header.stamp.to_sec()
        self.toc = tic

        ## current postion
        av = AXIS_DIRECTION * interface.angle_vector() * np.pi / 180
        robot_model.angle_vector(av)
        pos = robot_model.grasp_point.worldpos()

        dpos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]) * 0.005
        # ori_quat = np.array([
        #      msg.pose.orientation.w,
        #      msg.pose.orientation.x,
        #      msg.pose.orientation.y,
        #      msg.pose.orientation.z,
        # ])

        print(pos + dpos)

        print(robot_model.angle_vector())

        target_pos = pos + dpos

        ### IK

        ik = robot_model.inverse_kinematics(
            target_coords=Coordinates(pos=target_pos),
            # thre=[0.01],
            rotation_axis=False,
            avoid_weight_gain=0.0,
            move_target=robot_model.grasp_point)
        
        if ik is False:
            print("error")
            return None
        else:
            _av = ik

        # print("pos = {}".format(pos))
        # target = robot_model.grasp_point.copy_worldcoords()
        # print(i, target.worldpos(), ik)
        # print(robot_model.grasp_point.copy_worldcoords())
            
        # interface.angle_vector()
            
        ## move!!
        
        print(_av * 180 / np.pi)
        print(_av)
        target_av = AXIS_DIRECTION * _av * 180 / np.pi
        # interface.angle_vector(target_av[:-1], interface.search_servo_ids()[:-1])


rc = RobotController()
diffposition_subs = rospy.Subscriber("/iphone_aligned", PoseStamped, rc.update_endpos)

rospy.spin()