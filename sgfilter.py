#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from multiimu.msg import SavitzkyGolayCoeff  # 新しいメッセージ型のインポート
from collections import deque, defaultdict

class SavitzkyGolayFilter:
    def __init__(self, M, N):
        self.poly_deg = N
        self.half_datalength = M
        self.queue_size = 2 * M + 1
        self.imu_data_queues = defaultdict(lambda: deque(maxlen=self.queue_size))
        self.imu_timestamps_queues = defaultdict(lambda: deque(maxlen=self.queue_size))

        self.coeff_pub = rospy.Publisher('/imu_sgcoeff', SavitzkyGolayCoeff, queue_size=10)
        
        rospy.init_node('imu_data_listener')
        rospy.Subscriber('/imu', Imu, self.imu_callback)

    def imu_callback(self, msg):
        # Convert the message to np.array and get the timestamp
        imu_data = self.convert_to_array(msg)
        timestamp = msg.header.stamp.to_sec()

        # rospy.logwarn(msg.header.stamp.to_nsec())
        
        frame_id = msg.header.frame_id
        # Append data to respective queues
        self.imu_data_queues[frame_id].append(imu_data)
        self.imu_timestamps_queues[frame_id].append(timestamp)

        rospy.loginfo(f"Added IMU data and timestamp to queues for frame_id '{frame_id}'")

        # After adding new data, attempt to calculate coefficients and publish
        self.publish_coefficients(frame_id)

    def convert_to_array(self, msg):
        # Create an np.array with angular and linear accelerations
        return np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

    def is_data_ready(self, frame_id):
        """Check if there is enough data for the specified frame_id."""
        if len(self.imu_data_queues[frame_id]) < self.queue_size:
            return False
        return (frame_id in self.imu_data_queues)

    def calc_polynomial_coeffs(self, frame_id):
        if not self.is_data_ready(frame_id):
            rospy.logwarn(f"Not enough data to calculate coefficients for frame_id '{frame_id}'")
            return None

        # Extract timestamps and IMU data from the queues
        t_list = np.array(self.imu_timestamps_queues[frame_id])
        imu_data = np.vstack(self.imu_data_queues[frame_id])

        # Focusing on the middle timestamp
        mid_idx = len(t_list) // 2
        focusing_t = t_list[mid_idx]
        shift_t = t_list - focusing_t

        rospy.logwarn(shift_t)

        # Create matrix A for polynomial fitting
        A = np.vstack([shift_t ** i for i in range(self.poly_deg + 1)]).T

        rospy.logwarn(f"A.shape = {A.shape}")
        rospy.logwarn(f"imu_data.shape = {imu_data.shape}")
    
        # Calculate polynomial coefficients using least squares
        coeffs = np.linalg.pinv(A) @ imu_data

        return coeffs

    def publish_coefficients(self, frame_id):
        """Calculate and publish the polynomial coefficients."""
        coeffs = self.calc_polynomial_coeffs(frame_id)
        if coeffs is not None:
            # Create a SavitzkyGolayCoeff message
            coeff_msg = SavitzkyGolayCoeff()
            coeff_msg.header.stamp = rospy.Time.now()
            coeff_msg.header.frame_id = frame_id
            coeff_msg.polynomial_degree = self.poly_deg
            coeff_msg.half_datalength = self.half_datalength
            coeff_msg.timelist = np.array(self.imu_timestamps_queues[frame_id]).tolist()
            coeff_msg.coefflist = coeffs.flatten().tolist()  # Flatten the coefficients for the message

            rospy.logwarn(f"coeff_msg.polynomial_degree = {coeff_msg.polynomial_degree}")
            rospy.logwarn(f"coeff_msg.half_datalength = {coeff_msg.half_datalength}")
            rospy.logwarn(f"len(coeff_msg.coefflist) = {len(coeff_msg.coefflist)}")
            rospy.logwarn(f"coeffs.shape = {coeffs.shape}")
            rospy.logwarn(f"self.queue_size = {self.queue_size}")

            rospy.logwarn(np.array(self.imu_timestamps_queues[frame_id]).tolist())
            
            # Publish the message
            self.coeff_pub.publish(coeff_msg)
            rospy.loginfo(f"Published coefficients for frame_id '{frame_id}'")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    filter = SavitzkyGolayFilter(M=7, N=11)
    filter.run()
