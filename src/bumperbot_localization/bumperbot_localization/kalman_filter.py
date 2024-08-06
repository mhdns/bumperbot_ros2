#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class KalmanFilter(Node):

    def __init__(self):
        super().__init__("kalman_filter")

        self.odom_sub_ = self.create_subscription(Odometry, "/bumperbot_controller/odom_noisy", self.odomCallback, 10)
        self.imu_sub_ = self.create_subscription(Imu, "/imu/out", self.imuCallback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/bumperbot_controller/odom_kalman", 10)

        self.mean_ = 0.0
        self.variance_ = 1000.0
        self.imu_angular_z_ = 0.0
        self.is_first_odom_ = True
        self.last_angular_z_ = 0.0

        self.motion_ = 0.0
        self.kalman_odom_ = Odometry()

        self.motion_variance_ = 4.0
        self.measurement_variance_ = 0.5

    def imuCallback(self, imu):
        self.imu_angular_z_ = imu.angular_velocity.z

    def statePrediction(self):
        self.mean_ self.mean_ + self.motion_
        self.variance_ = self.variance_ + self.motion_variance_

    def measurementUpdate(self):
        self.mean_ = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_angular_z_)/(self.variance_ + self.measurement_variance_)
        self.variance_ = (self.variance_ * self.measurement_variance_) / (self.variance_ + self.measurement_variance_)

    def odomCallback(self, odom):
        self.kalman_odom_ = odom

        if self.is_first_odom_:
            self.is_first_odom_ = False
            self.mean_ = odom.twist.twist.angular.z
            self.last_angular_z_ = odom.twist.twist.angular.z
            return

        self.motion_ = odom.twist.twist.angular.z - self.last_angular_z_

        self.statePrediction()
        self.measurementUpdate()

        self.kalman_odom_.twist.twist.angular.z = self.mean_
        self.odom_pub.publish(self.kalman_odom_)

        self.last_angular_z_ = odom.twist.twist.angular.z

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()