# import native modules
import time
import os
from collections import deque
import csv
# import ROS2
import rclpy
from rclpy.node import Node
from gps_msgs.msg import GPSFix

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

# self defined functions

# define data saving path
if not os.path.exists("GPS_data"):
    os.mkdir("GPS_data")
trail_time = time.strftime("%Y%m%d%H%M%S")
# self.id_directory = os.path.join("data/image_details", f"{setrail_time}nav.csv")
sync_info_dir = os.path.join("GPS_data", f"{trail_time}.csv")
header = ['NAV_time', 'MAV_lat', 'NAv_long', 'NAV_heading', 'NAV_speed', 'RTK_time', 'RTK_lat',
          'RTK_long', 'RTK_yaw', 'RTK_pitch', 'RTK_roll', 'RTK_speed', 'err_lat', 'err_long', 'err_yaw',
          'err_pitch', 'err_roll', 'err_speed', 'RTK_cov']
if not os.path.exists(sync_info_dir):
    with open(sync_info_dir, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)


class MinimalPublisher(Node):
    def __init__(self, api):
        super().__init__('minimal_publisher')
        self.NAV_deque = deque(maxlen=20)
        self.NAV_msg = {}
        self.api = api
        self.api.nav_subscribe(self.nav_callback)
        self.rtk_sub = self.create_subscription(GPSFix, '/novatel/oem7/gps', self.rtk_callback, 100)
        self.i = 0

    def nav_callback(self, data: NavNotifData):
        # global sync_buffer
        self.NAV_msg = {
            "timestamp": data.timestamp,
            "latitude": data.latitude / 10e6,
            "longitude": data.longitude / 10e6,
            # "altitude": data.altitude / 10e3,
            "heading": data.heading,
            "speed": data.speed / 1000,
            # "satellites": data.number_of_used_satellites
        }
        NAV_values = list(self.NAV_msg.values())
        self.NAV_deque.append(NAV_values)
        # print(NAV_values)
        
    def rtk_callback(self, data):
        sec = data.header.stamp.sec
        nano_sec = data.header.stamp.nanosec

        # rtk infomation
        milliseconds = nano_sec // 1_000_000  # integer division to get first 3 digits
        # Combine seconds and milliseconds
        timestamp = float(f"{sec}{milliseconds:03d}")
        # print(timestamp)
        lat = data.latitude
        long = data.longitude
        speed = data.speed
        yaw = data.track
        roll = data.roll
        pitch = data.pitch

        # rtk error
        err_lat = data.err_horz
        err_long = data.err_vert
        err_speed = data.err_speed
        err_yaw = data.err_track
        err_roll = data.err_roll
        err_pitch = data.err_pitch

        # info_covariance
        pos_cov = data.position_covariance
        NAV_times = [item[0] for item in self.NAV_deque]
        if NAV_times:
            min_diff = float('inf')
            cloest_time = 0
            for nav_time in NAV_times:
                diff = abs(timestamp - nav_time)
                if diff < min_diff:
                    min_diff = diff
                    cloest_time = nav_time
            index = NAV_times.index(cloest_time)
            NAV_info = list(self.NAV_deque[index])
            data_save = NAV_info + [timestamp, lat, long, yaw, pitch, roll, speed, err_lat, err_long, err_yaw, err_pitch, err_roll, err_speed, pos_cov]
        # header = ['NAV_time', 'MAV_lat', 'NAv_long', 'NAV_heading', 'NAV_speed', 'RTK_time', 'RTK_lat',
        #   'RTK_long', 'RTK_yaw', 'RTK_pitch', 'RTK_roll', 'RTK_speed', 'err_lat', 'err_long', 'err_yaw',
        #   'err_pitch', 'err_roll', 'err_speed', 'RTK_cov']
        # Log to CSV
            with open(sync_info_dir, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(data_save)


def main(args=None):
    with create_cms_api(host='192.168.3.54') as api:
        rclpy.init(args=args)
        minimal_publisher = MinimalPublisher(api)
        rclpy.spin(minimal_publisher)
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
                     
