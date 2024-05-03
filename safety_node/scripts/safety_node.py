#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        # TODO: create ROS subscribers and publishers.

        # Subscribers for LaserScan and Odometry data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        
        # Publisher for emergency braking and drive commands
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.bool_pub = self.create_publisher(Bool, '/emergency_braking', 10)
        

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):

        # TODO: calculate TTC
        r = np.array(scan_msg.ranges)
        angles_increm = scan_msg.angle_increment
        beam_angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, angles_increm, dtype=float)

        # Compute the range rate
        r_dot = -self.speed * np.cos(beam_angles)

        # iTTC calculation using safe division and ensuring r_dot is negative
        safe_r_dot = np.maximum(-r_dot, 1e-6)  # Use a small positive constant to avoid division by zero
        iTTC = r / safe_r_dot  # Here, we don't need to condition on r < 0 since r is always positive

        threshold = 0.8
        emergency_braking = np.any(iTTC < threshold)  # Check if any iTTC is below the threshold

        # Prepare the Bool message and publish it
        emergency_braking_msg = Bool()
        emergency_braking_msg.data = bool(emergency_braking)  # Explicitly convert to bool to avoid AssertionError
        self.bool_pub.publish(emergency_braking_msg)

        # Act on emergency braking if needed
        if emergency_braking:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            self.get_logger().info("Emergency Braking engaged!")
            self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()