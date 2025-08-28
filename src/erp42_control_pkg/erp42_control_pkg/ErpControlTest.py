#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from erp42_interfaces_pkg.msg import ErpCmdMsg
# ros2 run erp42_test_pkg erp42_test_pub --ros-args -p gear:=1 -p steer:=0 -p speed:=0 -p brake:=1

class TestPublisher(Node):
    def __init__(self):
        super().__init__('erp42_test_pub')

        self.declare_parameter('gear', 1)
        self.declare_parameter('steer', 0)
        self.declare_parameter('speed', 0)
        self.declare_parameter('brake', 1)

        # 파라미터 값 가져오기
        self.gear = self.get_parameter('gear').value
        self.steer = self.get_parameter('steer').value
        self.speed = self.get_parameter('speed').value
        self.brake = self.get_parameter('brake').value

        # 퍼블리셔
        self.pub = self.create_publisher(ErpCmdMsg, '/erp42_ctrl_cmd', 10)

        # 10Hz 타이머
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            f"Start publishing: gear={self.gear}, steer={self.steer}, speed={self.speed}, brake={self.brake}"
        )

    def timer_callback(self):
        msg = ErpCmdMsg()
        msg.gear = int(self.gear)
        msg.steer = int(self.steer)
        msg.speed = int(self.speed)
        msg.brake = int(self.brake)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

