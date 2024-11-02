#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import Imu

import pypkg.src.multiimu_proc.dataclass as dataclass

def generate_random_msg(time):
    msg = Imu()

    msg.header.frame_id = "test{}".format(np.random.randint(3))
    msg.header.stamp = rospy.Time(time)

    msg.angular_velocity.x = np.random.randn()
    msg.angular_velocity.y = np.random.randn()
    msg.angular_velocity.z = np.random.randn()

    msg.linear_acceleration.x = np.random.randn()
    msg.linear_acceleration.y = np.random.randn()
    msg.linear_acceleration.z = np.random.randn()

    return msg

_t = 0
for _ in range(10):
    _t += np.random.rand() * 10**3
    msg = generate_random_msg(_t)
    imu = dataclass.ImuData.from_rosmsg(msg)
    print(imu)
    del imu