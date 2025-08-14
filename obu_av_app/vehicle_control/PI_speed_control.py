#! /usr/bin/env python3
# native imports
import csv
import os
import time
# need to pip install numpy, ROS2, and pycmssdk
import numpy as np
# import CV2X
from pycmssdk import (
    Asn1Type,
    FacMsgType,
    FacNotifData,
    asn1_decode,
    create_cms_api,
    NavDriveDirection,
    NavFix,
    NavNotifData,
    NavSetManual,
    NavSource,
    UtcTimestampMs
)
# import ROS2
import rclpy
from rclpy.node import Node
from ds_dbw_msgs.msg import UlcCmd
from std_msgs.msg import Bool

# saving directory
if not os.path.exists("data"):
    os.mkdir("data")
trial_time = time.strftime("%Y%m%d%H%M%S")
nav_directory = os.path.join("data", f"{trial_time}nav.csv")
nav_header = ['time', 'lat', 'long', 'alt', 'heading', 'speed', 'sat']
if not os.path.exists(nav_directory):
    with open(nav_directory, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(nav_header)
ctrl_directory = os.path.join("data", f"{trial_time}ctrl.csv")
ctrl_header = [
    'ctrl_time', 'ego_lat', 'ego_lon', 'ego_speed', 'target_speed',
    'Kp', 'Ki', 'dv', 'engaged'
]
if not os.path.exists(ctrl_directory):
    with open(ctrl_directory, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(ctrl_header)


def sign(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0


class PI_control(Node):
    def __init__(self, api, target_speed=10.0, Kp=0.5, Ki=0.1):
        super().__init__('PI_control')
        self.dt = 0.1  # control loop time step
        self.create_timer(self.dt, self.timer_callback)
        self.api = api
        self.api.nav_subscribe(self.nav_callback)
        self.t = 0
        self.enabled = False
        self.target_speed = target_speed
        self.Kp = Kp
        self.Ki = Ki
        self.integral_error = 0
        self.pre_err = 0
        self.ulc_cmd = UlcCmd()
        self.ulc_cmd.cmd_type = UlcCmd.CMD_ACCEL
        self.ulc_cmd.enable = True
        self.ulc_cmd.enable_shift_park = False
        self.NAV_msg = {}
        # Topics
        self.create_subscription(Bool, '/vehicle/dbw_enabled', self.recv_enable, 1)
        self.pub_ulc_cmd = self.create_publisher(UlcCmd, '/vehicle/ulc/cmd', 1)

        # Parameters
        self.ulc_cmd.enable_shift = self.declare_parameter('enable_shift', False).value  # Enable shifting between non-Park gears
        self.ulc_cmd.limit_accel = self.declare_parameter('limit_accel', 0.0).value  # Override default acceleration limit
        self.ulc_cmd.limit_decel = self.declare_parameter('limit_decel', 0.0).value  # Override default acceleration limit

    def nav_callback(self, data: NavNotifData):
        self.NAV_msg = {
            "timestamp": data.timestamp,
            "latitude": data.latitude / 10e6,
            "longitude": data.longitude / 10e6,
            "altitude": data.altitude / 10e3,
            "heading": data.heading,
            "speed": data.speed / 1000,
            "satellites": data.number_of_used_satellites}
        
        with open(nav_directory, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(list(self.NAV_msg.values()))

    def recv_enable(self, msg: Bool):
        if msg.data and not self.enabled:
            self.t = 0

        self.enabled = msg.data

    def timer_callback(self):
        if self.NAV_msg:
            ctrl_time = self.NAV_msg['timestamp']
            ego_lat = self.NAV_msg['latitude']
            ego_lon = self.NAV_msg['longitude']
            ego_speed = self.NAV_msg['speed']
            if self.enabled:
                error = self.target_speed - ego_speed
                print("target :", self.target_speed)
                print("ego speed :", ego_speed)
                print('real-time error is :', error)
                print('Integral_error is :', self.integral_error)
                if sign(error) != sign(self.pre_err):
                    self.integral_error = 0
                self.integral_error += error * self.dt
                dv = self.Kp * error + self.Ki * self.integral_error
                engaged = 1
                control_input = [ctrl_time, ego_lat, ego_lon, ego_speed, self.target_speed, self.Kp, self.Ki, dv, engaged]
                with open(ctrl_directory, 'a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(control_input)
                self.ulc_cmd.cmd = dv
                self.pub_ulc_cmd.publish(self.ulc_cmd)
                self.pre_err = error
            else:
                error = self.target_speed - ego_speed
                dv = self.Kp * error + self.Ki * self.integral_error
                engaged = 0
                control_input = [ctrl_time, ego_lat, ego_lon, ego_speed, self.target_speed, self.Kp, self.Ki, dv, engaged]
                with open(ctrl_directory, 'a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(control_input)

                dbw_enable_ulc_cmd = UlcCmd()
                dbw_enable_ulc_cmd.enable = True
                dbw_enable_ulc_cmd.cmd_type = UlcCmd.CMD_NONE
                self.pub_ulc_cmd.publish(dbw_enable_ulc_cmd)
