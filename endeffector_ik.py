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

from urdf_generator import AppleKinovo, SymbolicParameter, SymbolicParameterList

symparam_list = SymbolicParameterList([
    SymbolicParameter("link1", "link2", np.random.randn(3), np.random.randn(3)),
    SymbolicParameter("link2", "link3", np.random.randn(3), np.random.randn(3))
])

urdf_str = AppleKinovo(symparam_list).make_urdf()

print(urdf_str)

out_urdfpath = '/tmp/robot.urdf'
with open(out_urdfpath, 'w') as f:
    f.write(urdf_str)

robot_model = RobotModel()
robot_model.load_urdf_file(open(out_urdfpath))

## initialize
av = (1 - 2*np.random.rand(6)) * np.pi * 0.67

for i, _dpos in enumerate(np.random.rand(100,3)):
  robot_model.angle_vector(av)
  pos = robot_model.grasp_point.worldpos()
  dpos = _dpos / np.linalg.norm(_dpos) * 0.01

  target_pos = pos + dpos

  ik = robot_model.inverse_kinematics(
      target_coords=Coordinates(pos=target_pos),
      # thre=[0.01],
      rotation_axis=False,
      avoid_weight_gain=0.0,
      move_target=robot_model.grasp_point)
  
  if ik is False:
     print("error")
     continue
  else:
     av = ik

     

  # array([-0.3183119 ,  0.63661397,  0.        ], dtype=float32)
  print("pos = {}".format(pos))
  target = robot_model.grasp_point.copy_worldcoords()
  print(i, target.worldpos(), ik)
  # if type(ik) is bool and not ik:
  #     # raise ValueError
  #     continue
  # else:
  # print("pos = {}".format(pos))
  # robot_model.joint0.joint_angle(ik[0])
  # robot_model.joint1.joint_angle(ik[1])
  # print(robot_model.grasp_point.copy_worldcoords())
  # break
  # viewer.redraw()
  #<Coordinates 0x176d61c50 1.900 0.000 0.000 / 0.0 0.3 0.0>

  # print(dir(target))
  # robot_model.angle_vector(ik)
  # ik2 = robot_model.inverse_kinematics(
  #       target_coords=Coordinates(pos=target.worldpos(), rot=target.rpy_angle()[0] * np.array([1,1,0])),
  #       rotation_axis=True,
  #       avoid_weight_gain=0.0,
  #       move_target=robot_model.grasp_point)

  # print(pos, ik2)
  print(robot_model.grasp_point.copy_worldcoords())