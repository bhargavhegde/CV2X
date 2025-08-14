#! /usr/bin/env python3
# cv2x imports
from pycmssdk import create_cms_api
# ROS2 imports
import rclpy
# self developed modules
from PI_speed_control import PI_control

target_speed = 10
Kp = 0.8
Ki = 0.01


def main(args=None):
    with create_cms_api(host='192.168.3.54') as api:
        rclpy.init()
        node = PI_control(api, target_speed, Kp, Ki)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
