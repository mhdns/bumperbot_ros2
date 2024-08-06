#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class KalmanFilter(Node):

    def __init__(self):
        super().__init__("kalman_filter")

        self.odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom_noisy", self.odomCallback, 10)
        self.imu_sub_ = self.create_subscription(Imu, "imu/data", self.imuCallback, 10)
        self.odom_pub = self.create_publisher(Odometry, "bumperbot_controller/odom_kalman", 10)

        self.mean_ = 0.0
        self.variance_ = 1000.0
        self.imu_angular_z_ = 0.0
        self.is_first_odom_ = True
        self.last_angular_z_ = 0.0

        self.motion_ = 0.0
        self.kalman_odom = Odometry()

    def imuCallback(self, imu):
        self.imu_angular_z_ = imu.angular_velocity.z


    def odomCallback(self, odom):
        self.kalman_odom = odom

        if self.is_first_odom_:
            self.is_first_odom_ = False
            self.mean_ = odom.twist.twist.angular.z
            self.last_angular_z_ = odom.twist.twist.angular.z
            return

        self.statePrediction()
        self.measurementUpdate()

    def statePrediction(self):
        pass
    def measurementUpdate(self):
        pass
