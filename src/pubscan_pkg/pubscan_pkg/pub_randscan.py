#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pubscan_pkg_msgs.msg import LaserScanmock
import math
import random
from std_msgs.msg import Header

ANGLE_MIN_DEG = 0
ANGLE_MAX_DEG = 359
ANGLE_INCREMENT_DEG = 1
NUM_POINTS = 360
RANGE_MIN = 0.12
RANGE_MAX = 3.5

class LaserScanmockPublisher(Node):
    def __init__(self):
        super().__init__('LSmock_publisher')

        # Publisher
        self.publisher = self.create_publisher(LaserScanmock, '/ls_mock', 10)

        # 2초 주기 타이머
        self.timer = self.create_timer(2.0, self.timer_callback)

        self.get_logger().info("2초마다 랜덤 패턴 LaserScanmock 퍼블리시 시작")

        # 패턴 목록
        self.AVAILABLE_PATTERNS = ["front_wall", "left_wall", "right_wall"]

    # ----------------------- 기본 빈 스캔 생성 -----------------------
    def create_empty_scan(self):
        ranges = [float(RANGE_MAX) for _ in range(NUM_POINTS)]
        intensities = [100.0 for _ in range(NUM_POINTS)]

        scan = {
            "angle_min": math.radians(ANGLE_MIN_DEG),
            "angle_max": math.radians(ANGLE_MAX_DEG),
            "angle_increment": math.radians(ANGLE_INCREMENT_DEG),
            "range_min": RANGE_MIN,
            "range_max": RANGE_MAX,
            "ranges": ranges,
            "intensities": intensities
        }
        return scan

    # ----------------------- 벽 생성 로직 -----------------------
    def make_the_wall(self, ranges, center_deg, width_deg):
        half_width = width_deg // 2
        for offset in range(-half_width, half_width + 1):
            idx = (center_deg + offset) % NUM_POINTS
            ranges[idx] = 0.4

    def pattern_front_wall(self, scan):
        self.make_the_wall(scan["ranges"], center_deg=0, width_deg=40)

    def pattern_left_wall(self, scan):
        self.make_the_wall(scan["ranges"], center_deg=90, width_deg=30)

    def pattern_right_wall(self, scan):
        self.make_the_wall(scan["ranges"], center_deg=270, width_deg=30)

    # ----------------------- 단일 스캔 생성 -----------------------
    def generate_single_scan(self, pattern_name):
        scan = self.create_empty_scan()

        if pattern_name == "front_wall":
            self.pattern_front_wall(scan)
        elif pattern_name == "left_wall":
            self.pattern_left_wall(scan)
        elif pattern_name == "right_wall":
            self.pattern_right_wall(scan)

        return scan

    # ----------------------- 타이머 콜백 -----------------------
    def timer_callback(self):

        # 랜덤 패턴 선택
        pattern = random.choice(self.AVAILABLE_PATTERNS)

        scan_dict = self.generate_single_scan(pattern)

        # LaserScanmock 메시지 구성
        msg = LaserScanmock()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "lidar_link"

        msg.angle_min = float(scan_dict["angle_min"])
        msg.angle_max = float(scan_dict["angle_max"])
        msg.angle_increment = float(scan_dict["angle_increment"])
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        msg.ranges = scan_dict["ranges"]
        msg.intensities = scan_dict["intensities"]

        # publish
        self.publisher.publish(msg)

        self.get_logger().info(f"패턴 [{pattern}] 퍼블리시 완료")


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanmockPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
