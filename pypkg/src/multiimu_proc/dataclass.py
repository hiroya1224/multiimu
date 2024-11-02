#!/usr/bin/env python3
import numpy as np

class ImuData:
    def __init__(self, frame_id, time, accl, gyro) -> None:
        self.frame_id = frame_id
        self.time = time
        self.accl = accl
        self.gyro = gyro

    def __repr__(self) -> str:
        repr_strs = [
            f"<{self.__class__.__module__}.{self.__class__.__name__} object of",
            "  frame_id: {}".format(self.frame_id),
            "  time: {}".format(self.time),
            "  accl: {}".format(self.accl),
            "  gyro: {}".format(self.gyro),
            f"at {hex(id(self))}>"
        ]
        return "\n".join(repr_strs)

    @staticmethod
    def to_ndarray(msg_vector3):
        return np.array([
            msg_vector3.x,
            msg_vector3.y,
            msg_vector3.z,
        ])

    @classmethod
    def from_rosmsg(cls, msg): ## input: sensor_msgs/Imu
        frame_id = msg.header.frame_id
        time = msg.header.stamp.to_sec()
        accl = cls.to_ndarray(msg.linear_acceleration)
        gyro = cls.to_ndarray(msg.angular_velocity)
        return cls(frame_id, time, accl, gyro)
    

class ImuDataset:
    def __init__(self, frame_id) -> None:
        self.frame_id = frame_id
        self.data_list = []

    def update(self, data):
        self.data_list.append(data)
        self.data_list.pop(0)

        data.frame_id


class MultiImuDataList:
    def __init__(self):
        ImuDataList




