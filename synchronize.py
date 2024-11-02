#!/usr/bin/env python3
import rospy
import numpy as np
from multiimu.msg import SavitzkyGolayCoeff
from collections import defaultdict, deque
from copy import copy

class DataContainer:
    def __init__(self, size=7, poly_deg=5):
        self.size = size
        self.poly_deg = poly_deg
        self.mid_idx = size // 2
        self.time_stamps = deque([0.0] * size, maxlen=size)
        self.coeffs = deque([np.zeros(6)] * size, maxlen=size)

    def update(self, time_stamp, coeffs):
        self.time_stamps.append(time_stamp)
        self.coeffs.append(coeffs)

    def get_sorted_data(self):
        sorted_indices = sorted(range(len(self.time_stamps)), key=lambda i: self.time_stamps[i])
        sorted_times = [self.time_stamps[i] for i in sorted_indices]
        sorted_coeffs = [self.coeffs[i] for i in sorted_indices]
        return sorted_times, sorted_coeffs

class ImuInterpolator:
    @staticmethod
    def calculate_polynomial_coeffs(times, data, degree):
        mid_time = times[len(times) // 2]
        shifted_times = np.array(times) - mid_time
        A = np.vstack([shifted_times ** i for i in range(degree + 1)]).T
        coeffs = np.linalg.pinv(A) @ np.array(data)
        return coeffs

    @staticmethod
    def interpolate_value(coeffs, delta_t):
        time_powers = np.array([delta_t ** i for i in range(len(coeffs))])
        return coeffs.T @ time_powers

class CoeffSubscriber:
    def __init__(self):
        rospy.init_node('sg_coeff_subscriber')
        self.coeff_subscriber = rospy.Subscriber('/imu_sgcoeff', SavitzkyGolayCoeff, self.callback)
        self.coeff_queues = defaultdict(lambda: deque(maxlen=10))
        self.data_containers = defaultdict(lambda: DataContainer())

    def callback(self, msg):
        frame_id = msg.header.frame_id
        time_stamp = msg.header.stamp.to_sec()
        
        # Extract coefficients from the message
        coeffs = np.array(msg.coefflist).reshape(-1, 6)  # Assuming coefflist represents 6 variables' polynomial coefficients

        self.data_containers[frame_id].update(time_stamp, coeffs)
        self.coeff_queues[frame_id].append(msg)
        rospy.loginfo(f"Stored message for frame ID '{frame_id}'")

        oldest_frame_id = self.get_oldest_frame_id()
        if oldest_frame_id:
            central_timestamp_oldest = self.get_central_timestamp(oldest_frame_id)
            rospy.loginfo(f"Central timestamp for frame ID '{oldest_frame_id}': {central_timestamp_oldest}")
            self.perform_interpolation(central_timestamp_oldest)

    def get_oldest_frame_id(self):
        if self.coeff_queues:
            return min(self.coeff_queues, key=lambda k: min(msg.header.stamp.to_sec() for msg in self.coeff_queues[k]))
        return None

    def get_central_timestamp(self, frame_id):
        if frame_id in self.coeff_queues and self.coeff_queues[frame_id]:
            timelist = self.coeff_queues[frame_id][-1].timelist
            return timelist[len(timelist) // 2]
        return None

    def perform_interpolation(self, base_time):
        for frame_id, container in self.data_containers.items():
            times, coeffs = container.get_sorted_data()
            if base_time < times[0] or base_time > times[-1]:
                rospy.loginfo(f"Base time {base_time} is out of range for frame ID '{frame_id}'")
                continue

            coeffs_poly = ImuInterpolator.calculate_polynomial_coeffs(times, coeffs, container.poly_deg)
            delta_t = base_time - times[container.mid_idx]

            interpolated_value = ImuInterpolator.interpolate_value(coeffs_poly, delta_t)
            rospy.loginfo(f"Interpolated value for frame ID '{frame_id}': {interpolated_value}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    subscriber = CoeffSubscriber()
    subscriber.run()
