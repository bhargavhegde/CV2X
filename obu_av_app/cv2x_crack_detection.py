#! /usr/bin/env python3
'''
This file should contain the main application logic for CV2X Crack Detection.
'''

# Standard library imports
import argparse
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
from obu_api import run_obu
from process_crack_image import save_images

#, crack_gps_lat, crack_gps_lon

class LincolnTest(Node):
    def __init__(self, api, acc, test_type, ts_filename):
        super().__init__('lincoln_test')

        # Controlling Publisher Frequency
        self.create_timer(0.02, self.acceleration_control_pub)
        self.create_timer(0.02, self.crack_callback)

        # including the CV2X subscription
        self.api = api
        # self.api.fac_subscribe(FacMsgType.FAC_MSG_US_BSM, self.fac_callback)
        self.api.nav_subscribe(self.nav_callback)

        self.curr_vel = 0
        self.input_acc = 0
        self.target_acc = acc

        if(test_type == 's'):
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
        
        # TODO: Set these parameters as config.json and load them.
        # Camera Intrinsic parameters (new calibration 07/30/2025)
        self.f_x = 3406.175114 # fx in meters
        self.c_x = 1080.429855 # Principal point x
        self.f_y = 3423.223242 # fy in meters
        self.c_y = 785.284572 # Principal point y

        # Camera extrinsic parameters
        self.camera_height = 1.28 #1.48  # Camera height above road in meters
        self.camera_pitch = 0  # Camera tilt downward in radians
        # TODO: Need an Azimuth angle for the camera, for now just assuming 0 radians (facing forward and aligned with heading)
        self.camera_yaw = 0  # Camera yaw angle in radians

        self.image_width = 2064  # Image width in pixels
        self.image_height = 1544  # Image height in pixels
        
        # Earth radius in meters
        self.R = 6371000.0  # Earth radius in meters

        # Latest camera GPS and heading
        self.camera_lat = None
        self.camera_lon = None
        self.heading = None  # In radians

        #RTK GPS
        self.rtk_flag = False #True #False
        self.rtk_lat = None
        self.rtk_lon = None
        self.rtk_heading = None


    # for subscribing current speed
    def ulc_report(self, msg: UlcReport):
        # Timestamp
        timestamp = Clock().now().to_msg()
        self.curr_vel = msg.vel_meas

        log_msg = f"TS_sec: {timestamp.sec}, TS_nsec: {timestamp.nanosec}, \
                    Acceleration Measurement: {msg.accel_meas}, \
                    Velocity Measurement: {msg.vel_meas}, Enabled: {self.enabled}\n"
        
        filename_ulc = os.path.join('data','ulc_report_logger_' + self.ts_filename + '.txt')
        with open(filename_ulc, "a") as log_file:
            log_file.write(log_msg)
        # self.get_logger().info(log_msg)

    def nav_callback(self, data: NavNotifData) -> None:
        print('here!')
        self.own_speed = data.speed / 100
        self.camera_lat = data.latitude / 10e6
        self.camera_lon = data.longitude / 10e6
        self.heading = math.radians(data.heading / 10e2)
        # Timestamp
        timestamp = Clock().now().to_msg()
        filename_nav = os.path.join('data','nav_report_logger_' + self.ts_filename + '.txt')
        log_msg = f"TS_sec: {timestamp.sec}, TS_nsec: {timestamp.nanosec}, Velocity Measurement: {self.own_speed}, Enabled: {self.enabled}, Longitude: {self.camera_lon}, Latitude: {self.camera_lat}, Heading: {self.heading}\n"
        # print(log_msg)
        with open(filename_nav, "a") as log_file:
            log_file.write(log_msg)
        self.get_logger().info(log_msg) 

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
            if(self.curr_vel <= 0.0):
                self.flag_static = True
            if(self.flag_static):
                self.ulc_cmd.cmd =  self.target_acc # Write the acceleration to be tested
            else:
                self.ulc_cmd.cmd =  -0.5
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
        if angle<0:
            angle += 360 # For ex: -10 will become 350
        return angle

    def rtk_callback(self, info):
        self.rtk_lat = info.latitude
        self.rtk_lon = info.longitude
        #self.rtk_heading = math.radians(self.angle_check(info.track))
    
    def inspva_callback(self, msg: INSPVA):
        # Azimuth is heading in degrees from INS fused solution
        self.rtk_heading = math.radians(msg.azimuth)

    def crack_callback(self):
        '''
            This function does the conversion of GPS coordinates to pixel coordinates
            TODO: the math needs double check, a few factors appears to be missing
                - vehicle GPS location is not at the camera but at the antenna.
                - camera azimuth angle is not considered
            TODO: refactor this code as a separate function that can be tested and reused.
            TODO: takes crack GPS coordinates from external sources (CV2X)
        '''
        # print('crack callback is working!!!')

        # (temp) Hard code the crack GPS location for demonstration purposes        
        # This corresponds to a location at the Governor Lot, measured by RTK GPS antenna.

        crack_lat =  43.0041665901 # Latitude in degrees
        crack_lon = -78.7905901382 # Longitude in degrees

        # This corresponds to a location near the parking lot, measured by vehicle (rough estimation)
        # crack_lat =  43.00282684637123  #43.00282684637123 #43.002785  # Latitude in degrees
        # crack_lon = -78.78670267223183  #-78.78670267223183 #-78.7867654 # Longitude in degrees
 
        if self.camera_lat is None or self.camera_lon is None or self.heading is None:
            self.get_logger().warn('Camera GPS or heading not received yet')
            return

        if None in [self.f_x, self.f_y, self.c_x,
                    self.c_y, self.camera_height, self.camera_pitch,
                    self.image_width, self.image_height]:
            self.get_logger().warn('Camera parameters not set')
            return

        #Convert GPS to local Cartesian coordinates (For RTK GPS)
        if(self.rtk_flag):
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
        if(self.rtk_flag):
            cos_psi = math.cos(self.rtk_heading)
            sin_psi = math.sin(self.rtk_heading)
        else:
            #Rotate to car frame (y-axis = heading, x-axis = heading + 90 degrees)
            cos_psi = math.cos(self.heading)
            sin_psi = math.sin(self.heading)

        x_prime = x * cos_psi - y * sin_psi
        y_prime = x * sin_psi + y * cos_psi
        z_prime = z

        # Transform to camera coordinates
        # assuming that Camera is at (0, 0, camera_height) with pitch (downward tilt of the camera)
        cos_pitch = math.cos(self.camera_pitch)
        sin_pitch = math.sin(self.camera_pitch)

        # Camera coordinate transformation (accounting for pitch)
        X_c = x_prime  # Left-Right
        Y_c = y_prime*sin_pitch + (z_prime-self.camera_height)*cos_pitch  # Up-Down
        Z_c = y_prime*cos_pitch + (z_prime-self.camera_height)*sin_pitch  # Depth, where camera is viewing

        pixel_msg = Float32MultiArray() #Int32MultiArray()
        u, v, x1, y1, x2, y2 = -1, -1, -1, -1, -1, -1
        # Project to image plane (pinhole camera model)
        #Z_c = -Z_c
        if Z_c > 0:

            # Pixel coordinates (u, v)
            u = self.f_x * (X_c/Z_c) + self.c_x
            v = (-1*self.f_y*(Y_c/Z_c)) + self.c_y # Negative due to image y-axis pointing downward
            #v = self.image_height - v # Flipping the image

            # Rounding u and v
            u = round(u, 0)
            v = round(v,0)

            #Bounds check using image_width and image_height
            if not (0 < u < self.image_width and 0 < v < self.image_height):
                pixel_msg.data = [x1, y1, x2, y2, self.rtk_lat, self. rtk_lon, self.rtk_heading]
                #self.get_logger().warn(f'Crack at (u={u:.2f}, v={v:.2f}) is outside image bounds (0,0,{self.image_width},{self.image_height})')
            else:
                x1, x2, y1, y2 = self.get_box(u,v)
                pixel_msg.data = [x1, y1, x2, y2, self.rtk_lat, self. rtk_lon, self.rtk_heading]

        else:
            self.get_logger().warn('Crack is behind camera, cannot project')
            pixel_msg.data = [x1, y1, x2, y2, self.rtk_lat, self. rtk_lon, self.rtk_heading]

        # Publish pixel coordinates

        self.pixel_pub.publish(pixel_msg)
        self.get_logger().info(f'Z_c: {Z_c}, Crack Pixel: u={u}, v={v} Border Values: x1={x1}, x2={x2}, y1={y1}, y2={y2}')

        # Please note that the origin of u,v lies on the top left of the image. u is increasing towards the right and v is increasing towards the bottom.


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Takes in target acceleration and the type of test to be executed 'r' for rolling and 's' for static")
    parser.add_argument('acceleration', type=float)
    parser.add_argument('test_type', type=str)
    args = parser.parse_args()
    ts_filename = Clock().now().to_msg().sec

    # Image Queue
    image_queue = queue.Queue()
    crack_gps_lat = None
    crack_gps_lon = None

    # receive_and_respond()
    # threading.Thread(target=receive_and_respond(), daemon=True).start()

    with create_cms_api(host='192.168.3.54') as api:
        rclpy.init()
        node = LincolnTest(api, args.acceleration, args.test_type, str(ts_filename)) #, crack_gps_lat, crack_gps_lon
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        # Defining Threads
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        thread_obu = threading.Thread(target=run_obu, args=(image_queue,), daemon=True) #, crack_gps_lat, crack_gps_lon
        save_images_thread = threading.Thread(target=save_images, args=(image_queue,), daemon=True)

        # Initiating Threads
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
