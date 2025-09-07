#! /usr/bin/env python3
'''
This file should contain the main application logic for CV2X Crack Detection.
'''

# Standard library imports
import argparse
import json
import math
import os
import queue
import threading
import time

# Third-party library imports
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.executors import MultiThreadedExecutor

# Drive by Wire (DBW) imports
from ds_dbw_msgs.msg import UlcCmd, UlcReport
from ds_dbw_msgs.msg import VehicleVelocity
from std_msgs.msg import Bool, Int32MultiArray, Float32MultiArray

# RTK GPS imports
from sensor_msgs.msg import NavSatFix
from gps_msgs.msg import GPSFix
from novatel_oem7_msgs.msg import INSPVA

# CV2X import
from pycmssdk import ( # noqa: F401
    Asn1Type,
    FacMsgType,
    FacNotifData,
    NavDriveDirection,
    NavFix,
    NavNotifData,
    NavSetManual,
    NavSource,
    UtcTimestampMs,
    asn1_decode,
    create_cms_api
)

# Local application imports
from obu_api import run_obu, get_data_obu
from process_crack_image import save_images

# GPS location
global crack_lat, crack_lon, test_num

crack_lat = 1000
crack_lon = 1000

# Test Number
test_num = 27

dir_path = f"data/Test_{test_num}"
os.makedirs(dir_path, exist_ok=True)


class LincolnTest(Node):
    def __init__(self, api, acc, test_type, ts_filename):
        super().__init__('lincoln_test')

        # Controlling Publisher Frequency
        # [CRH] is 50 Hz to fast? What if we just run when ever a new image is received?
        self.create_timer(0.02, self.acceleration_control_pub)
        self.create_timer(0.02, self.crack_callback)

        # including the CV2X subscription
        self.api = api
        # self.api.fac_subscribe(FacMsgType.FAC_MSG_US_BSM, self.fac_callback)
        self.api.nav_subscribe(self.nav_callback)

        self.curr_vel = 0
        self.input_acc = 0
        self.target_acc = acc

        if (test_type == 's'):
            self.flag_static = False
            self.ts_filename = 'static_a=' + str(acc) + '_' + ts_filename
        else:
            self.flag_static = True
            self.ts_filename = 'rolling_a=' + str(acc) + '_' + ts_filename

        self.t = 0
        self.enabled = False

        self.ulc_cmd = UlcCmd()
        self.ulc_cmd.cmd_type = UlcCmd.CMD_ACCEL
        self.ulc_cmd.enable = True
        self.ulc_cmd.enable_shift_park = False

        # Subscription
        self.create_subscription(Bool, '/vehicle/dbw_enabled', self.recv_enable, 1)
        self.create_subscription(UlcReport, '/vehicle/ulc/report', self.ulc_report, 1)
        self.rtk_sub = self.create_subscription(GPSFix, '/novatel/oem7/gps', self.rtk_callback, 100)
        # Subscribe to inspva for azimuth
        self.inspva_subscription = self.create_subscription(INSPVA, '/novatel/oem7/inspva', self.inspva_callback, 50)

        # Publisher
        self.pub_ulc_cmd = self.create_publisher(UlcCmd, '/vehicle/ulc/cmd', 1)
        # TODO: ISSUE the precision of neither Float32MultiArray nor Int32MultiArray is enough for RTK GPS
        # which is 2 + 9 decimal places = 11.
        self.pixel_pub = self.create_publisher(Int32MultiArray, '/crack_pixel', 10)

        # Parameters
        self.v1 = self.declare_parameter('v1', 0.0).value  # Speed 1
        self.v2 = self.declare_parameter('v2', 5.0).value  # Speed 2
        self.period = self.declare_parameter('period', 15.0).value # Period of wave pattern
        self.ulc_cmd.enable_shift = self.declare_parameter('enable_shift', False).value  # Enable shifting between non-Park gears
        self.ulc_cmd.limit_accel = self.declare_parameter('limit_accel', 0.0).value  # Override default acceleration limit
        self.ulc_cmd.limit_decel = self.declare_parameter('limit_decel', 0.0).value  # Override default acceleration limit

        # Area of Interest Parameters

        # Target crack location
        # self.crack_loc_lat = crack_gps_lat
        # self.crack_loc_lon = crack_gps_lon

        # Load camera parameters from config file
        self._load_camera_config()

        # Latest camera GPS and heading
        self.camera_lat = None
        self.camera_lon = None
        self.heading = None  # In radians

        # RTK GPS
        self.rtk_flag = True
        self.rtk_lat = None
        self.rtk_lon = None
        self.rtk_heading = None
        self.z_distance = 10
        # pixel coordinates message counter
        self.count = 0

        log_param = f"f_x: {self.f_x}, c_x: {self.c_x}, c_y: {self.c_y}, f_y: {self.f_y}, camera_height: {self.camera_height}, camera_pitch: {self.camera_pitch}, image_width: {self.image_width}, image_height: {self.image_height}, Radius_Earth: {self.R}"

        filename_param = os.path.join(dir_path, 'param_log.txt')

        with open(filename_param, "a") as log_file:
            log_file.write(log_param + "\n")

    def _load_camera_config(self):
        """Load camera configuration parameters from config/camera.json file."""
        try:
            # Get the directory where this script is located
            script_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(script_dir, 'config', 'camera.json')
            
            with open(config_path, 'r') as f:
                config = json.load(f)
            
            # Load camera intrinsic parameters
            intrinsic = config['camera_intrinsic']
            self.f_x = intrinsic['f_x']  # fx in pixels
            self.c_x = intrinsic['c_x']  # Principal point x
            self.f_y = intrinsic['f_y']  # fy in pixels
            self.c_y = intrinsic['c_y']  # Principal point y
            self.image_width = intrinsic['image_width']  # Image width in pixels
            self.image_height = intrinsic['image_height']  # Image height in pixels
            
            # Load camera extrinsic parameters
            extrinsic = config['camera_extrinsic']
            self.camera_height = extrinsic['camera_height']  # Camera height above road in meters
            self.camera_pitch = extrinsic['camera_pitch']  # Camera tilt downward in radians
            self.camera_yaw = extrinsic['camera_yaw']  # Camera yaw angle in radians
            
            # Load earth constants
            earth = config['earth_constants']
            self.R = earth['radius']  # Earth radius in meters
            
            self.get_logger().info(f'Loaded camera configuration from {config_path}')
            
        except FileNotFoundError:
            self.get_logger().error(f'Camera config file not found at {config_path}.')
            raise # Re-raise the caught exception
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing camera config JSON: {e}.')
            raise # Re-raise the caught exception
        except KeyError as e:
            self.get_logger().error(f'Missing key in camera config: {e}.')
            raise  # Re-raise the caught exception

    # for subscribing current speed
    def ulc_report(self, msg: UlcReport):
        # Timestamp
        timestamp = Clock().now().to_msg()
        self.curr_vel = msg.vel_meas

        log_msg = f"TS_sec: {timestamp.sec}, TS_nsec: {timestamp.nanosec}, \
                    Acceleration Measurement: {msg.accel_meas}, \
                    Velocity Measurement: {msg.vel_meas}, Enabled: {self.enabled}\n"

        filename_ulc = os.path.join(dir_path, 'ulc_report_logger_' + self.ts_filename + '.txt')
        with open(filename_ulc, "a") as log_file:
            log_file.write(log_msg)
        # self.get_logger().info(log_msg)

    def nav_callback(self, data: NavNotifData) -> None:
        '''
        (TODO) Remove this if we move cv2x API call out of this class
        '''
        # print('here!')
        self.own_speed = data.speed / 100
        self.camera_lat = data.latitude / 10e6
        self.camera_lon = data.longitude / 10e6
        self.heading = math.radians(data.heading / 10e2)
        # Timestamp
        timestamp = Clock().now().to_msg()
        filename_nav = os.path.join(dir_path, 'nav_report_logger_' + self.ts_filename + '.txt')
        log_msg = f"TS_sec: {timestamp.sec}, TS_nsec: {timestamp.nanosec}, Velocity Measurement: {self.own_speed}, Enabled: {self.enabled}, Longitude: {self.camera_lon}, Latitude: {self.camera_lat}, Heading: {self.heading}\n"
        # print(log_msg)
        with open(filename_nav, "a") as log_file:
            log_file.write(log_msg)
        # self.get_logger().info(log_msg)

    def recv_enable(self, msg: Bool):
        if msg.data and not self.enabled:
            self.t = 0

        self.enabled = msg.data

    def acceleration_control_pub(self):
        '''
        Publish acceleration control commands.
        TODO: need to upgrade this part to include connected cruise control
        '''
        if self.enabled:
            # [CRH] This logic is confusing. 
            # Does it always bring the vehicle to stop first and then start from static??
            if self.curr_vel <= 0.0:
                self.flag_static = True
                self.ulc_cmd.cmd = self.target_acc # Write the acceleration to be tested
            else:
                self.ulc_cmd.cmd = -0.5
            self.input_acc = self.ulc_cmd.cmd
            self.pub_ulc_cmd.publish(self.ulc_cmd)
        else:
            dbw_enable_ulc_cmd = UlcCmd()
            dbw_enable_ulc_cmd.enable = True
            dbw_enable_ulc_cmd.cmd_type = UlcCmd.CMD_NONE
            self.pub_ulc_cmd.publish(dbw_enable_ulc_cmd)

    # AOI Calculation related functions
    def get_box(self, u, v, box_size=512):
        half_box = box_size // 2

        # Initial box centered at (u, v)
        x1 = u - half_box
        x2 = u + half_box
        y1 = v - half_box
        y2 = v + half_box

        # Adjust box if it goes out of image boundaries
        if x1 < 0:
            x1 = 0
            x2 = box_size
        elif x2 > self.image_width:
            x2 = self.image_width
            x1 = self.image_width - box_size

        if y1 < 0:
            y1 = 0
            y2 = box_size
        elif y2 > self.image_height:
            y2 = self.image_height
            y1 = self.image_height - box_size

        # Ensure the box stays within bounds
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(self.image_width, x2)
        y2 = min(self.image_height, y2)

        return int(x1), int(x2), int(y1), int(y2)

    def angle_check(self, angle):
        if angle < 0:
            angle += 360  # For ex: -10 will become 350
        return angle

    def rtk_callback(self, info):
        self.rtk_lat = info.latitude
        self.rtk_lon = info.longitude
        print(f'RTK_lat; {self.rtk_lat}')
        print(f'RTK_lon; {self.rtk_lon}')
        # self.rtk_heading = math.radians(self.angle_check(info.track))

    def inspva_callback(self, msg: INSPVA):
        # Azimuth is heading in degrees from INS fused solution
        self.rtk_heading = math.radians(msg.azimuth)

    def crack_callback(self):
        '''
        [CRH] this is the main function that determine the Cropping windows according to GPS and heading.
        It is called at a fixed rate and take the latest GPS and heading as inputs, and output the pixel
        Q: Should we coordinate the value with respect to the image time?
            This function does the conversion of GPS coordinates to pixel coordinates
            TODO: the math needs double check, a few factors appears to be missing
                - vehicle GPS location is not at the camera but at the antenna.
                - camera azimuth angle is not considered
            TODO: refactor this code as a separate function that can be tested and reused.
            TODO: takes crack GPS coordinates from external sources (CV2X)
        '''

        # Get crack GPS coordinates from CV2X
        crack_lat, crack_lon = get_data_obu()
        print("crack_lat: ", crack_lat)
        print("crack_lon: ", crack_lon)
        if self.camera_lat is None or self.camera_lon is None or self.heading is None:
            self.get_logger().warn('Camera GPS or heading not received yet')
            return

        if None in [self.f_x, self.f_y, self.c_x,
                    self.c_y, self.camera_height, self.camera_pitch,
                    self.image_width, self.image_height]:
            self.get_logger().warn('Camera parameters not set')
            return

        # log the gps
        timestamp = Clock().now().to_msg()
        log_aoi_msg = f"TS_sec: {timestamp.sec}, TS_nsec: {timestamp.nanosec}, \
                    crack_lat: {crack_lat}, \
                    crack_lon: {crack_lon}\n"

        filename_aoi = os.path.join(dir_path, 'aoi_info_logger_' + self.ts_filename + '.txt')
        with open(filename_aoi, "a") as log_file:
            log_file.write(log_aoi_msg)

        # Convert GPS to local Cartesian coordinates (For RTK GPS)
        # TODO: use Haversine or pyproj package instead
        if self.rtk_flag:
            delta_phi = math.radians(crack_lat - float(self.rtk_lat))  # Latitude difference in radians
            delta_lambda = math.radians(crack_lon - float(self.rtk_lon))  # Longitude difference in radians
            phi_avg = math.radians((float(self.rtk_lat) + crack_lat) / 2)  # Average latitude in radians
        else:
            # Convert GPS to local Cartesian coordinates
            delta_phi = math.radians(crack_lat - self.camera_lat)  # Latitude difference in radians
            delta_lambda = math.radians(crack_lon - self.camera_lon)  # Longitude difference in radians
            phi_avg = math.radians((self.camera_lat + crack_lat) / 2)  # Average latitude in radians

        # North-South (y) and East-West (x) distances
        '''
            TODO: For projection we can use pyproj (https://pyproj4.github.io/pyproj/stable/)
            usage: Convert lat/lon to local ENU coordinates (meters)
            from pyproj import Proj, transform
            x, y = transform(self.proj_wgs84, self.proj_enu, lon, lat)
        '''
        y = self.R * delta_phi  # Meters
        x = self.R * delta_lambda * math.cos(phi_avg)  # Meters
        z = 0.0  # Crack is on the road

        # For RTK GPS
        if self.rtk_flag:
            cos_psi = math.cos(self.rtk_heading)
            sin_psi = math.sin(self.rtk_heading)
        else:
            cos_psi = math.cos(self.heading)
            sin_psi = math.sin(self.heading)

        x_prime = x * cos_psi - y * sin_psi
        y_prime = x * sin_psi + y * cos_psi
        z_prime = z

        # Transform to camera coordinates
        cos_pitch = math.cos(self.camera_pitch)
        sin_pitch = math.sin(self.camera_pitch)

        # Camera coordinate transformation (accounting for pitch)
        X_c = x_prime
        Y_c = y_prime * sin_pitch + (z_prime - self.camera_height) * cos_pitch
        Z_c = y_prime * cos_pitch + (z_prime - self.camera_height) * sin_pitch

        pixel_msg = Int32MultiArray()
        u, v, x1, y1, x2, y2 = -1, -1, -1, -1, -1, -1
        # Project to image plane (pinhole camera model)
        # [TODO] make the logic a separate function that can be tested and reused.
        if Z_c > 0:

            if Z_c <= self.z_distance:

                u = self.f_x * (X_c / Z_c) + self.c_x
                v = (-1 * self.f_y * (Y_c / Z_c)) + self.c_y

                u = round(u, 0)
                v = round(v, 0)

                if not (0 < u < self.image_width and 0 < v < self.image_height):
                    pixel_msg.data = [x1, y1, x2, y2, self.count]
                else:
                    x1, x2, y1, y2 = self.get_box(u, v)
                    pixel_msg.data = [x1, y1, x2, y2, self.count]

            else:
                dist_left = Z_c - self.z_distance
                print(f"Will start to generate mask in {dist_left}m")
                pixel_msg.data = [x1, y1, x2, y2, self.count]

        else:
            self.get_logger().warn('Crack is behind camera, cannot project')
            pixel_msg.data = [x1, y1, x2, y2, self.count]

        # Publish pixel coordinates
        self.count += 1

        self.pixel_pub.publish(pixel_msg)

        log_main_msg = f"Count:{self.count}, Lat:{self.rtk_lat}, Lon:{self.rtk_lon}, Heading: {self.rtk_heading},Border Values: x1={x1}, x2={x2}, y1={y1}, y2={y2}\n"

        filename_send = os.path.join(dir_path, 'main_log.txt')

        with open(filename_send, "a") as log_file:
            log_file.write(log_main_msg)

        # Please note that the origin of u,v lies on the top left of the image. u is increasing towards the right and v is increasing towards the bottom.


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Takes in target acceleration and the type of test to be executed 'r' for rolling and 's' for static")
    parser.add_argument('--acceleration', type=float, default=0.0, required=False)
    parser.add_argument('--test_type', type=str, default='s', required=False)
    args = parser.parse_args()
    ts_filename = Clock().now().to_msg().sec

    image_queue = queue.Queue()
    crack_gps_lat = None
    crack_gps_lon = None

    with create_cms_api(host='192.168.3.54') as api:
        rclpy.init()
        node = LincolnTest(api, args.acceleration, args.test_type, str(ts_filename))
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        thread_obu = threading.Thread(target=run_obu, args=(image_queue, test_num), daemon=True)
        save_images_thread = threading.Thread(target=save_images, args=(image_queue, test_num), daemon=True)

        ros_thread.start()
        time.sleep(3)
        save_images_thread.start()
        time.sleep(3)
        thread_obu.start()

        try:
            ros_thread.join()
            save_images_thread.join()
            thread_obu.join()
        except KeyboardInterrupt:
            print("Done!!!")
        finally:
            print("Shutting down...")
            node.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
            save_images_thread.join()
