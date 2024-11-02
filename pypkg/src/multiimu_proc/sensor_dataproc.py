#!/usr/bin/env python3
import rospy
import numpy as np
from multiimu.msg import SavitzkyGolayCoeff
from collections import defaultdict, deque

class CoeffSubscriber:
    def __init__(self):
        rospy.init_node('sg_coeff_subscriber')
        self.coeff_subscriber = rospy.Subscriber('/imu_sgcoeff', SavitzkyGolayCoeff, self.callback)
        
        # Dictionary to store messages by frame_id
        self.coeff_queues = defaultdict(lambda: deque(maxlen=10))

    def callback(self, msg):
        frame_id = msg.header.frame_id
        time_stamp = msg.header.stamp.to_sec()

        # Store the received message
        self.coeff_queues[frame_id].append(msg)
        rospy.loginfo(f"Stored message for frame ID '{frame_id}'")

        # Process oldest frame's central timestamp
        oldest_frame_id = self.get_oldest_frame_id()
        if oldest_frame_id:
            central_timestamp_oldest = self.get_central_timestamp(oldest_frame_id)
            rospy.loginfo(f"Central timestamp for frame ID '{oldest_frame_id}': {central_timestamp_oldest}")

            # Perform interpolation using this timestamp
            self.perform_interpolation(central_timestamp_oldest)

    def get_oldest_frame_id(self):
        """Retrieve the frame ID with the oldest timestamp."""
        if self.coeff_queues:
            return min(self.coeff_queues, key=lambda k: min(msg.header.stamp.to_sec() for msg in self.coeff_queues[k]))
        return None

    def get_central_timestamp(self, frame_id):
        """Retrieve the central timestamp of the timelist for the latest message in the given frame ID."""
        if frame_id in self.coeff_queues and self.coeff_queues[frame_id]:
            timelist = self.coeff_queues[frame_id][-1].timelist
            return timelist[len(timelist) // 2]
        return None

    def perform_interpolation(self, base_time):
        """Perform interpolation for each frame ID based on the base_time central timestamp."""
        for frame_id, queue in self.coeff_queues.items():
            closest_msg = self.get_closest_message_to_central_timestamp(frame_id, base_time)
            
            if closest_msg:
                t_list = closest_msg.timelist
                # Reshape coefflist based on half_datalength and polynomial_degree
                half_datalength = closest_msg.half_datalength
                polynomial_degree = closest_msg.polynomial_degree
                coeffs = np.array(closest_msg.coefflist).reshape(half_datalength, polynomial_degree + 1)
                
                # Check if base_time is within the t_list range
                if t_list[0] <= base_time <= t_list[-1]:
                    delta_t = base_time - t_list[len(t_list) // 2]

                    # Perform interpolation using polynomial coefficients
                    interpolated_value = self.interpolate_using_coeffs(coeffs, delta_t)
                    rospy.loginfo(f"Interpolated value for frame ID '{frame_id}': {interpolated_value}")
                else:
                    rospy.loginfo(f"Base time {base_time} is out of range for frame ID '{frame_id}'")

    def get_closest_message_to_central_timestamp(self, frame_id, central_timestamp):
        """Find the message in the queue for the specified frame ID with the closest central timestamp."""
        closest_msg = None
        min_difference = float('inf')
        
        for msg in self.coeff_queues[frame_id]:
            msg_central_timestamp = msg.timelist[len(msg.timelist) // 2]
            difference = abs(msg_central_timestamp - central_timestamp)
            if difference < min_difference:
                min_difference = difference
                closest_msg = msg

        return closest_msg

    def interpolate_using_coeffs(self, coeffs, delta_t):
        """Interpolate values using the provided polynomial coefficients and time difference."""
        time_powers = np.array([delta_t ** i for i in range(coeffs.shape[1])])
        return coeffs @ time_powers  # Resulting in interpolated values

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    subscriber = CoeffSubscriber()
    subscriber.run()
