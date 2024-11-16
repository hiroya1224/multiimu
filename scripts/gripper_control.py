#!/usr/bin/env python3

import numpy as np
from std_msgs.msg import String
import json
import rospy

rospy.init_node("gripper_controller")

###### ARMH7 ##########

rospy.logwarn("initialize RCB4")

from rcb4.armh7interface import ARMH7Interface
import time

# if __name__ == "__main__":

## initialize
# av = (1 - 2*np.random.rand(6)) * np.pi * 0.67
# rospy.logwarn(f"servo ids = {servo_ids}")

class RobotController:

    def __init__(self):
        self.elapsed_time = 0
        self.toc = None
        self.prev_stamp = -1.

        self.interface = None
        self.servo_ids = None

    def init_interface(self):
        interface = ARMH7Interface()
        print(interface.auto_open())
        interface.hold()

        servo_ids = interface.search_servo_ids()

        self.interface = interface
        self.servo_ids = servo_ids

    def callback(self, _msg: String):
        msg = json.loads(_msg.data)

        control = msg["control"]
        action = msg["button_action"]

        rospy.loginfo(msg)

        # if control == "gripper_open_close":
        #     # interface.angle_vector([-60], [12])
        # print("gripper_open_close")
            
rc = RobotController()
diffposition_subs = rospy.Subscriber("/gripper_control", String, rc.callback)

rospy.spin()