#! /usr/bin/env python3
# native imports
import time
import os
from collections import deque
import csv
# need to pip install numpy, ROS2, and pycmssdk
import numpy as np
# import CV2X
from pycmssdk import(
     Asn1Type,
     FacMsgType,
     FacNotifData,
     asn1_decode,
     create_cms_api)
from pycmssdk import(# noqa: F401
     NavDriveDirection,
     NavFix,
     NavNotifData,
     NavSetManual,
     NavSource,
     UtcTimestampMs,
     create_cms_api)
# import ROS2
import rclpy
from rclpy.node import Node
from ds_dbw_msgs.msg import UlcCmd
from std_msgs.msg import Bool


def integrate_time(timestamp, hearing_secmark):
    floored_min = timestamp // 60000
    reminder_ms = timestamp % 60000
    if reminder_ms >= hearing_secmark:
        time_integrated = hearing_secmark + floored_min * 60000
    else:
        time_integrated = hearing_secmark + (floored_min - 1) * 60000
    return time_integrated


def generate_fac_time(timestamp, secmark):
    time_generated = integrate_time(timestamp, secmark)
    if abs(time_generated - timestamp) > 50000:
        time_generated = time_generated + 60000
        if time_generated > timestamp:
            time_generated = timestamp
    return time_generated


def finding_diffmin(array1, value):
    latest_msg = value
    array1_diff = [abs(item - latest_msg) for item in array1]
    min_value = min(array1_diff)
    return array1_diff.index(min_value), min_value


def syncing(N_BUFFER, F_BUFFER, S_BUFFER):
    N_profile = list(N_BUFFER['NAV_msg'])
    F_profile = list(F_BUFFER.values())
    N_time = [item[0] for item in N_profile]
    F_time = [item[0][-1] for item in F_profile]
    latest_FAC = min(F_time)
    if len(N_time) > 1:
        N_ind, _ = finding_diffmin(N_time, latest_FAC)

        if 'latest_N_time' not in S_BUFFER:
            S_BUFFER['latest_N_time'] = deque([N_time[0]], maxlen=20)
        else:
            S_BUFFER['latest_N_time'].appendleft(N_time[0])

        if 'S_N_profile' not in S_BUFFER:
            S_BUFFER['S_N_profile'] = deque([N_profile[N_ind]], maxlen=20)
        else:
            S_BUFFER['S_N_profile'].appendleft(N_profile[N_ind])

        for keys in F_BUFFER:
            F_history = list(F_BUFFER[keys])
            F_time_history = [item[-1] for item in F_history]
            F_ind, _ = finding_diffmin(F_time_history, N_time[N_ind])
            if keys not in S_BUFFER:
                S_BUFFER[keys] = deque([F_history[F_ind]], maxlen=20)
            else:
                S_BUFFER[keys].appendleft(F_history[F_ind])
        return S_BUFFER
    else:
        return {}


def headway_real(lat_ego, lon_ego, hearing_lat, hearing_lon):
    r = 6371000
    # e for point 1 , and f for point 2
    lat_e = lat_ego
    lon_e = lon_ego
    lat_f = hearing_lat
    lon_f = hearing_lon

    lat_er = np.radians(lat_e)
    lat_fr = np.radians(lat_f)

    lat_diff = lat_f - lat_e
    lon_diff = lon_f - lon_e

    lat_diffr = np.radians(lat_diff)
    lon_diffr = np.radians(lon_diff)

    intermediate = np.sqrt((1 - np.cos(lat_diffr) + np.cos(lat_er) * np.cos(lat_fr) * (1 - np.cos(lon_diffr))) / 2)
    haver_d = 2 * r * np.arctan(intermediate)

    return haver_d


# define data saving path
if not os.path.exists("data"):
    os.mkdir("data")
trail_time = time.strftime("%Y%m%d%H%M%S")
nav_directory = os.path.join("data", f"{trail_time}nav.txt")
fac_directory = os.path.join("data", f"{trail_time}fac.txt")
sync_directory = os.path.join("data", f"{trail_time}sync.txt")
ctrl_directory = os.path.join("data", f"{trail_time}ctrl.txt")

nav_directory = os.path.join("data", f"{trail_time}nav.csv")
nav_header = ['time', 'lat', 'long', 'alt', 'heading', 'speed', 'sat']
if not os.path.exists(nav_directory):
    with open(nav_directory, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(nav_header)

fac_directory = os.path.join("data", f"{trail_time}fac.csv")
fac_header = ['ID', 'msgCount', 'secMark', 'lat', 'long', 'elevation', 'speed', 'heading', 'angle', 'time']
if not os.path.exists(fac_directory):
    with open(fac_directory, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(fac_header)

sync_directory = os.path.join("data", f"{trail_time}sync.csv")
sync_header = ['latest_nav', 'sync_nav', 'sync_fac']
if not os.path.exists(sync_directory):
    with open(sync_directory, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(sync_header)

ctrl_directory = os.path.join("data", f"{trail_time}ctrl.csv")
ctrl_header = ['ctrl_time', 'ego_lat', 'ego_lon', 'v1_lat', 'v1_lon', 'D', 'D_nol', 'Vh', 'v', 'v1', 'v_dot', 'engaged']
if not os.path.exists(ctrl_directory):
    with open(ctrl_directory, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(ctrl_header)


class ACC(Node):
    def __init__(self, api):
        super().__init__('acc')
        self.create_timer(0.01, self.timer_callback)

        self.NAV_msg = {}
        self.FAC_msg = {}
        self.F_BUFFER = {}
        self.N_BUFFER = {}
        self.S_BUFFER = {}
        self.api = api
        self.api.fac_subscribe(FacMsgType.FAC_MSG_US_BSM, self.fac_callback)
        self.api.nav_subscribe(self.nav_callback)
        self.t = 0
        self.enabled = False
        self.ulc_cmd = UlcCmd()
        self.ulc_cmd.cmd_type = UlcCmd.CMD_ACCEL
        self.ulc_cmd.enable = True
        self.ulc_cmd.enable_shift_park = False

        # Topics
        self.create_subscription(Bool, '/vehicle/dbw_enabled', self.recv_enable, 1)
        self.pub_ulc_cmd = self.create_publisher(UlcCmd, '/vehicle/ulc/cmd', 1)
        # Add the Report part
        # self.create_subscription(Ulc_Report, '/vehicle/ulc/report', self.ulc_report 1)

        # Parameters
        self.v1 = self.declare_parameter('v1', 0.0).value  # Speed 1
        self.v2 = self.declare_parameter('v2', 5.0).value  # Speed 2
        self.period = self.declare_parameter('period', 10.0).value # Period of wave pattern
        self.ulc_cmd.enable_shift = self.declare_parameter('enable_shift', False).value  # Enable shifting between non-Park gears
        self.ulc_cmd.limit_accel = self.declare_parameter('limit_accel', 0.0).value  # Override default acceleration limit
        self.ulc_cmd.limit_decel = self.declare_parameter('limit_decel', 0.0).value  # Override default acceleration limit

    def nav_callback(self, data: NavNotifData):
        # global sync_buffer
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
        if 'NAV_msg' not in self.N_BUFFER:
            self.N_BUFFER['NAV_msg'] = deque([list(self.NAV_msg.values())], maxlen=10)
        else:
            self.N_BUFFER['NAV_msg'].appendleft(list(self.NAV_msg.values()))

        if not self.F_BUFFER:
            print('waiting for sync')
        else:
            self.S_BUFFER = syncing(self.N_BUFFER, self.F_BUFFER, self.S_BUFFER)

    def fac_callback(self, key: int, data: FacNotifData, buffer: bytes) -> None:
        print('here!')
        timestamp = round(time.time() * 1000)
        decoded_message = asn1_decode(buffer, Asn1Type.US_MESSAGE_FRAME)
        hearing_ID = decoded_message["value"][1]["coreData"]["id"].hex()
        FAC_timestamp = generate_fac_time(timestamp, decoded_message["value"][1]["coreData"]["secMark"])
        self.FAC_msg = {
            "OBU_ID": hearing_ID,
            #    "timestamp": timestamp,
            "msgCnt": decoded_message["value"][1]["coreData"]["msgCnt"],
            "secMark": decoded_message["value"][1]["coreData"]["secMark"],
            "latitude": decoded_message["value"][1]["coreData"]["lat"] / 10e6,
            "longitude": decoded_message["value"][1]["coreData"]["long"] / 10e6,
            "elevation": decoded_message["value"][1]["coreData"]["elev"] / 10e3,
            "speed": decoded_message["value"][1]["coreData"]["speed"] / 50,
            "heading": decoded_message["value"][1]["coreData"]["heading"],
            "angle": decoded_message["value"][1]["coreData"]["angle"],
            "FAC_timestamp": FAC_timestamp}
        with open(fac_directory, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(list(self.FAC_msg.values()))

        if hearing_ID not in self.F_BUFFER:
            self.F_BUFFER[hearing_ID] = deque([list(self.FAC_msg.values())], maxlen=10)
        else:
            self.F_BUFFER[hearing_ID].appendleft(list(self.FAC_msg.values()))

    def recv_enable(self, msg: Bool):
        if msg.data and not self.enabled:
            self.t = 0

        self.enabled = msg.data

    def timer_callback(self):
        sync_info = list(element[0] for element in self.S_BUFFER.values())
        if self.enabled:
            if sync_info:
                with open(sync_directory, 'a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(sync_info)
                ctrl_time = sync_info[0]
                ego_lat = sync_info[1][1]
                ego_lon = sync_info[1][2]
                v1_lat = sync_info[2][3]
                v1_lon = sync_info[2][4]
                v = sync_info[1][-2]
                v1 = sync_info[2][-4]
                # computing D:
                D = headway_real(ego_lat, ego_lon, v1_lat, v1_lon)
                D_nol = D - 5
                print(D)

                if D_nol < 5:
                    Vh = 0
                elif D_nol >= 5 and D_nol < 40:
                    Vh = 0.6 * (D_nol - 5)
                else:
                    Vh = 35

                v_dot = 0.4 * (Vh - v) + 0.6 * (v1 - v)
                engaged = 1
                control_input = list([ctrl_time, ego_lat, ego_lon, v1_lat, v1_lon, D, D_nol, Vh, v, v1, v_dot, engaged])
                with open(ctrl_directory, 'a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(control_input)

                self.ulc_cmd.cmd = v_dot
                self.pub_ulc_cmd.publish(self.ulc_cmd)

            else:
                pass
        else:
            if sync_info:
                with open(sync_directory, 'a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(sync_info)
                ctrl_time = sync_info[0]
                ego_lat = sync_info[1][1]
                ego_lon = sync_info[1][2]
                v1_lat = sync_info[2][3]
                v1_lon = sync_info[2][4]
                v = sync_info[1][-2]
                v1 = sync_info[2][-4]
                # computing D:
                D = headway_real(ego_lat, ego_lon, v1_lat, v1_lon)
                D_nol = D - 5
                print(D)

                if D_nol < 5:
                    Vh = 0
                elif D_nol >= 5 and D_nol < 40:
                    Vh = 0.6 * (D_nol - 5)
                else:
                    Vh = 35

                v_dot = 0.4 * (Vh - v) + 0.6 * (v1 - v)
                engaged = 0
                control_input = list([ctrl_time, ego_lat, ego_lon, v1_lat, v1_lon, D, D_nol, Vh, v, v1, v_dot, engaged])
                with open(ctrl_directory, 'a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(control_input)
            else:
                pass
        
            dbw_enable_ulc_cmd = UlcCmd()
            dbw_enable_ulc_cmd.enable = True
            dbw_enable_ulc_cmd.cmd_type = UlcCmd.CMD_NONE
            self.pub_ulc_cmd.publish(dbw_enable_ulc_cmd)


def main(args=None):
    with create_cms_api(host='192.168.3.54') as api:
        rclpy.init()
        node = ACC(api)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
