#!/usr/bin/env python3

import rclpy
from safety_module.safety_module_io_nanoscan import SafetyModuleIO


def main(args=None):
    rclpy.init(args=args)
    rc_node = SafetyModuleIO()
    rclpy.spin(rc_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
