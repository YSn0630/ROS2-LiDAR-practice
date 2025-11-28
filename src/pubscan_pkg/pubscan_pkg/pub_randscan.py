#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import math
import random

ANGLE_MIN_DEG = 0
ANGLE_MAX_DEG = 359
ANGLE_INCREMENT_DEG = 1
NUM_POINTS = 360
RANGE_MIN = 0.12
RANGE_MAX = 3.5

class LaserScanPublisher(Node):
    def __init__(self):
        super().__init__('scan_mock_publisher')

        # 구독: Python main.py에서 보낸 Twist
        self.sub_cmd = self.create_subscription(
            Twist,
            '/cmd_mock',
            self.cmd_callback,
            10
        )

        # 퍼블리셔들
        self.pub_scan = self.create_publisher(LaserScan, '/scan_mock', 10)
        self.pub_tt = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.latest_linear = 0.0
        self.latest_angular = 0.0

        self.timer = self.create_timer(2.0, self.timer_callback)

        self.get_logger().info("표준 LaserScan 사용 시작")

        self.patterns = ["front_path", "left_path", "right_path", "back_path"]

    def create_empty_scan(self):
        ranges = [float(RANGE_MAX) for _ in range(NUM_POINTS)]
        intensities = [100.0 for _ in range(NUM_POINTS)]
        return ranges, intensities

    def make_wall(self, ranges, center_deg, width_deg):
        half = width_deg // 2
        for i in range(-half, half+1):
            idx = (center_deg + i) % NUM_POINTS
            ranges[idx] = 0.25

    def generate_scan(self, pattern):
        ranges, intensities = self.create_empty_scan()

        if pattern == "front_path":
            self.make_wall(ranges, 90, 40)
            self.make_wall(ranges, 270, 40)
        elif pattern == "left_path":
            self.make_wall(ranges, 0, 40)
            self.make_wall(ranges, 90, 40)
        elif pattern == "right_path":
            self.make_wall(ranges, 0, 40)
            self.make_wall(ranges, 270, 40)
        elif pattern == "back_path":
            self.make_wall(ranges, 0, 40)
            self.make_wall(ranges, 90, 40)
            self.make_wall(ranges, 270, 40)

        return ranges, intensities

    def cmd_callback(self, msg: Twist):
        self.latest_linear = msg.linear.x
        self.latest_angular = msg.angular.z
        self.get_logger().info(f"cmd_mock 구독됨: linear={msg.linear.x}, angular={msg.angular.z}")

    def timer_callback(self):
        pattern = random.choice(self.patterns)
        ranges, intensities = self.generate_scan(pattern)

        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "lidar"

        msg.angle_min = math.radians(ANGLE_MIN_DEG)
        msg.angle_max = math.radians(ANGLE_MAX_DEG)
        msg.angle_increment = math.radians(ANGLE_INCREMENT_DEG)
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        msg.ranges = ranges
        msg.intensities = intensities

        self.pub_scan.publish(msg)
        self.get_logger().info(f"LaserScan pattern [{pattern}] 발행")

        # 터틀봇 제어도 같이 발행
        tmsg = Twist()
        tmsg.linear.x = self.latest_linear
        tmsg.angular.z = self.latest_angular
        self.pub_tt.publish(tmsg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
