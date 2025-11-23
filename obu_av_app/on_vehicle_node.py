# !/usr/bin/env python3
# Node written to run on vehicle, with help of copilot
# author: Haosong Xiao

# Standard library imports
# import argparse
import json
import os
import time

# Third-party library imports
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.executors import MultiThreadedExecutor

# Drive by Wire (DBW) imports
from ds_dbw_msgs.msg import UlcCmd, UlcReport
from ds_dbw_msgs.msg import VehicleVelocity
from std_msgs.msg import Bool, Int32MultiArray, Int64MultiArray, String

# RTK GPS imports
from gps_msgs.msg import GPSFix
from novatel_oem7_msgs.msg import INSPVA

# Local application imports
from quantification.pixel_conversion import Cameras_HX

# define folder name
from datetime import datetime
import glob

def create_auto_test_folder():

    base_dir = os.getcwd()
    data_dir = os.path.join(base_dir, 'data')
    if not os.path.exists(data_dir):
        os.makedirs(data_dir, exist_ok=True)

    today = datetime.now().strftime("%Y%m%d")
    pattern = os.path.join(data_dir, f"Test_{today}_*")
    existing_folders = glob.glob(pattern)
    
    if not existing_folders:
        test_num = f"{today}_001"
    else:
        numbers = []
        for folder in existing_folders:
            folder_name = os.path.basename(folder)
            try:
                parts = folder_name.split('_')
                if len(parts) >= 3:
                    num = int(parts[2])
                    numbers.append(num)
            except (ValueError, IndexError):
                continue

        if numbers:
            next_num = max(numbers) + 1
            test_num = f"{today}_{next_num:03d}"
        else:
            test_num = f"{today}_001"

    test_folder = os.path.join(data_dir, f"Test_{test_num}")
    os.makedirs(test_folder, exist_ok=True)

    return test_num

class OnVehicleNode(Node):
    def __init__(self):
        super().__init__('on_vehicle_node')

        init_time = self.get_clock().now()
        init_sec, init_nsec = init_time.seconds_nanoseconds()
        self.node_start_time = int((init_sec + init_nsec * 1e-9) * 1000)

        # subscribe rtk 
        self.create_subscription(GPSFix, '/novatel/oem7/gps', self.rtk_callback, 50)
        self.create_subscription(INSPVA, '/novatel/oem7/inspva', self.inspva_callback, 50)

        # subscribe ulc
        self.create_subscription(UlcReport, '/vehicle/ulc/report', self.ulc_report, 1)

        # publish indicator
        self.record_pub = self.create_publisher(Int64MultiArray, '/recording', 50)
        self.test_num_pub = self.create_publisher(String, '/test_number', 10)
        self.ready_to_process_pub = self.create_publisher(Int64MultiArray, '/ready_to_process', 10)

        # Create auto test folder and store the path
        self.test_num = create_auto_test_folder()
        self.test_folder = os.path.join(os.getcwd(), 'data', f"Test_{self.test_num}")
        self.get_logger().info(f"Using test folder: {self.test_folder}")
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        tunable_params_path = os.path.join(script_dir, 'quantification', 'config', 'tunable_params.json')
        config_path = os.path.join(script_dir, 'quantification', 'config', 'camera.json')
        
        # for real time gps
        self.vehicle_lat = None
        self.vehicle_lon = None
        self.vehicle_heading = None
        self.rtk_ros_gen = None
        self.rtk_ros_sub = None
        # self.rtk_ros_logged = None

        # for real time ulc
        self.ulc_gen = None
        self.ulc_sub = None # please note that we logged the ulc same time it's been sub 

        # for crack gps
        self.crack_lat = None
        self.crack_lon = None
        
        # for cropping window parameters
        self.f_x = None
        self.c_x = None
        self.f_y = None
        self.c_y = None
        self.camera_roll = None
        self.camera_pitch = None
        self.camera_yaw = None
        self.camera_height = None
        self.image_width = None
        self.image_height = None
        self.offset_X = None
        self.offset_Y = None

        # for indicating when to process
        self.dist = 1000
        self.ready_process = 0

        with open(tunable_params_path, 'r') as f:
            self.tunable_params = json.load(f)
        if self.tunable_params is not None:
            self.camera_pitch = self.tunable_params.get("camera_pitch", 0.0)
            self.camera_yaw = self.tunable_params.get("camera_yaw", 0.0)
        else:
            self.get_logger().warn("Using default tunable parameters due to loading failure")

        self.modelHX = Cameras_HX(config_path, self.camera_yaw, self.camera_pitch)
        if self.modelHX is not None:
            self.f_x = self.modelHX.camera_intrinsics['fx']
            self.c_x = self.modelHX.camera_intrinsics['cx']
            self.f_y = self.modelHX.camera_intrinsics['fy']
            self.c_y = self.modelHX.camera_intrinsics['cy']
            self.camera_roll = self.modelHX.camera_roll
            self.camera_height = self.modelHX.camera_height
            self.image_width = self.modelHX.image_width
            self.image_height = self.modelHX.image_height
            self.offset_X = self.modelHX.offset_x
            self.offset_Y = self.modelHX.offset_y
        else:
            self.get_logger().warn("Using default camera parameters due to model initialization failure")

        self.count = 0
        self.record = 0
        self.get_crack_coordinates()
        
        # One-time logging of all camera parameters and crack coordinates
        self.log_camera_parameters_once()
        
        self.create_timer(0.02, self.publish_test_num)  
        self.create_timer(0.02, self.detection_callback)

    def ulc_report(self, msg: UlcReport):
        # Timestamp
        now = self.get_clock().now().seconds_nanoseconds()
        now_time = now[0] + now[1] * 1e-9
        now_time = int(now_time * 1000)

        # self.curr_vel = msg.vel_meas

        log_msg = f"log time: {now_time}, \
                    acceleration: {msg.accel_meas}, \
                    velocity: {msg.vel_meas}\n"

        ulc_record_log = os.path.join(self.test_folder, 'ulc_report_log.txt')
        with open(ulc_record_log, 'a') as f:
                f.write(log_msg)
        # self.get_logger().info(log_msg)

    def get_crack_coordinates(self):
        # this can be replaced by using OBU node subcription
        self.crack_lat = 42.992613310789565 #42.992597586720315
        self.crack_lon = -78.79383332414315 #-78.79382606912415
    
    def log_camera_parameters_once(self):
        # tunable params, only need to log once
        camera_params_file = os.path.join(self.test_folder, 'camera_parameters.txt')
        with open(camera_params_file, 'w') as f:
            f.write(f"fx: {self.f_x}, fy: {self.f_y}, cx: {self.c_x}, cy: {self.c_y}, roll: {self.camera_roll}, pitch: {self.camera_pitch}, yaw: {self.camera_yaw}, height: {self.camera_height}, image width: {self.image_width}, image height: {self.image_height}, offset_x: {self.offset_X}, offset_y: {self.offset_Y}, crack_lat: {self.crack_lat}, crack_lon: {self.crack_lon}\n")
        
        self.get_logger().info(f"Camera parameters written to: {camera_params_file}")
    
    def rtk_callback(self, info):
        # [deg]

        msg_sec = info.header.stamp.sec
        msg_nsec = info.header.stamp.nanosec
        msg_time = msg_sec + msg_nsec * 1e-9
        now = self.get_clock().now().seconds_nanoseconds()
        now_time = now[0] + now[1] * 1e-9

        msg_time = int(msg_time * 1000)
        now_time = int(now_time * 1000)

        print(f'msg_generated: {msg_time}')
        print(f'msg_called: {now_time}')
        self.rtk_lat = info.latitude
        self.rtk_lon = info.longitude
        self.rtk_ros_gen = msg_time
        self.rtk_ros_sub = now_time
        # print(f'RTK_lat; {self.rtk_lat}')
        # print(f'RTK_lon; {self.rtk_lon}')

    def inspva_callback(self, msg: INSPVA):
        # [deg]
        self.rtk_heading = msg.azimuth

    def publish_test_num(self):
        """Publish the current test number periodically"""
        msg = String()
        msg.data = self.test_num
        self.test_num_pub.publish(msg)
    
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
    
    def detection_callback(self):

        # time_crop_generated = int(time.time() * 1000)
        time_crop_generated = self.get_clock().now().seconds_nanoseconds()
        time_crop_generated = time_crop_generated[0] + time_crop_generated[1] * 1e-9
        time_crop_generated = int(time_crop_generated * 1000)

        if self.vehicle_lat is not None and self.vehicle_lon is not None and self.vehicle_heading is not None:
            car_lat = self.vehicle_lat
            car_lon = self.vehicle_lon
            car_heading = self.vehicle_heading
        else:
            car_lat = self.rtk_lat
            car_lon = self.rtk_lon
            car_heading = self.rtk_heading

        if self.crack_lat is not None and self.crack_lon is not None:
            x_enu, y_enu = self.modelHX.gps_to_local(self.crack_lat, self.crack_lon, car_lat, car_lon)
            crack_car = self.modelHX.enu_to_car_frame([x_enu, y_enu], car_heading)
            crack_camera = self.modelHX.vehicle_to_camera_frame(crack_car)
            # print(f'crack camera: {crack_camera}')
            u, v = self.modelHX.project_to_pixel(crack_camera)
        else:
            crack_camera = [0, 0, 0]
            u, v = -1, -1

        record_msg = Int64MultiArray()
        process_msg = Int64MultiArray()
        if crack_camera[2] > 0 and crack_camera[2] < 20 and (0 < u < self.image_width and 0 < v < self.image_height):
        # if True:
            self.dist = crack_camera[2]
            self.record = 1
            x1, x2, y1, y2 = self.get_box(u, v)
        elif crack_camera[2] < 0:
            self.record = 0
            x1, x2, y1, y2 = -1, -1, -1, -1
            self.get_logger().info(f'dist: {self.dist}')
            if self.dist != 1000:
                self.ready_process = 1
            else:
                pass
            self.get_logger().info(f'process: {self.ready_process}')    
        else:
            self.record = 0
            self.ready_process = 0
            x1, x2, y1, y2 = -1, -1, -1, -1

        record_msg.data = [time_crop_generated, self.count, self.record]
        process_msg.data = [time_crop_generated, self.count, self.ready_process]
        self.record_pub.publish(record_msg)
        self.ready_to_process_pub.publish(process_msg)
        # Write detection data to txt file in the test folder
        detection_record_only = os.path.join(self.test_folder, 'detection_record_1_log.txt')
        if self.record == 1:
            with open(detection_record_only, 'a') as f:
                f.write(f"Node start time: {self.node_start_time}, timestamp: {time_crop_generated}, RTK gen_time: {self.rtk_ros_gen}, RTK_sub_time: {self.rtk_ros_sub}, count: {self.count}, car_lat: {car_lat:.6f}, car_lon: {car_lon:.6f}, heading: {car_heading:.6f}, record: {self.record}, x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}\n")
        else:
            pass
        all_detection_log_file = os.path.join(self.test_folder, 'all_detection_log.txt')
        with open(all_detection_log_file, 'a') as f:
            f.write(f"Node start time: {self.node_start_time}, timestamp: {time_crop_generated}, RTK_gen_time: {self.rtk_ros_gen}, RTK_sub_time: {self.rtk_ros_sub}, count: {self.count}, car_lat: {car_lat:.6f}, car_lon: {car_lon:.6f}, heading: {car_heading:.6f}, record: {self.record}, x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}\n")
        
        # update the count info
        self.count += 1

def main(args=None):
 
    rclpy.init(args=args)
    try:
        node = OnVehicleNode()
        node.get_logger().info("OnVehicleNode started successfully")
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("OnVehicleNode interrupted by user")
    except Exception as e:
        print(f"OnVehicleNode failed: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("OnVehicleNode shutdown complete")

if __name__ == '__main__':
    main()
    