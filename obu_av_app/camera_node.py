#!/usr/bin/env python3
# Node written to run on vehicle camera, with help of copilot
# author: Haosong Xiao

# native imports
import os
from typing import Optional
# need to pipinstall
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int64MultiArray

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info('Starting CameraNode...')

        init_time = self.get_clock().now()
        init_sec, init_nsec = init_time.seconds_nanoseconds()
        self.node_start_time = int((init_sec + init_nsec * 1e-9) * 1000)

        # Camera setup
        self.declare_parameter('camera_topic_prefix', '/vimbax_camera_')
        self.declare_parameter('camera_topic_suffix', '/image_raw')
        prefix = self.get_parameter('camera_topic_prefix').value
        suffix = self.get_parameter('camera_topic_suffix').value

        self.bridge = CvBridge()
        
        # State variables
        self.test_num = None
        self.test_folder = None
        self.test_base_folder = None  # Base test folder (not image subfolder)
        self.is_recording = False
        self.frame_count = 0
        self.current_image = None
        self.recording_timestamp = None  # Timestamp when recording status changed
        self.image_gen_time = None

        # Subscribe to test number from on_vehicle_node
        self.test_num_sub = self.create_subscription(
            String, '/test_number', self.test_num_callback, 50)
        
        # Subscribe to recording indicator from on_vehicle_node
        self.record_sub = self.create_subscription(
            Int64MultiArray, '/recording', self.record_callback, 50)

        # Find and subscribe to camera topic
        input_topic = self.find_image_topic(prefix, suffix)
        if input_topic is None:
            raise RuntimeError('Camera topic not found')

        self.image_sub = self.create_subscription(
            Image, input_topic, self.image_callback, 20)
        self.get_logger().info(f'Subscribed to camera topic: {input_topic}')
        self.get_logger().info('Waiting for test number and recording signals...')

    def test_num_callback(self, msg: String):
        """Handle test number updates from on_vehicle_node."""
        self.test_num = msg.data
        self.setup_test_folder()
        # self.get_logger().info(f'Updated test number to: {self.test_num}')

    def setup_test_folder(self):
        """Create test folder structure for image logging."""
        if self.test_num is None:
            return
            
        base = os.path.dirname(os.path.realpath(__file__))
        test_str = f"Test_{self.test_num}"
        self.test_base_folder = os.path.join(base, 'data', test_str)
        self.test_folder = os.path.join(self.test_base_folder, 'image_folder')
            
        # Create both base test folder and image subfolder
        os.makedirs(self.test_folder, exist_ok=True)
        os.makedirs(self.test_base_folder, exist_ok=True)
        os.chmod(self.test_folder, 0o777)
        os.chmod(self.test_base_folder, 0o777)
        
        # self.get_logger().info(f'Test folder ready: {self.test_folder}')

    def record_callback(self, msg: Int64MultiArray):
        """Handle recording state updates from on_vehicle_node."""
        if len(msg.data) >= 3:  # Ensure we have at least 3 elements
            self.is_recording = msg.data[2] == 1  
            # print(f'flag of recording: {self.is_recording}')
            self.recording_timestamp = msg.data[0]  
            status = "started" if self.is_recording else "stopped"
            # self.get_logger().info(f'Recording {status}')
        else:
            self.get_logger().warn(f'Recording message has insufficient data: {len(msg.data)} elements, expected at least 3')

    def find_image_topic(self, prefix: str, suffix: str) -> Optional[str]:
        """Find the camera image topic."""
        for topic, types in self.get_topic_names_and_types():
            if topic.startswith(prefix) and topic.endswith(suffix) and \
               'sensor_msgs/msg/Image' in types:
                return topic
        return None

    def image_callback(self, msg: Image):
        """Process incoming camera images."""
        try:
            msg_sec = msg.header.stamp.sec
            msg_nsec = msg.header.stamp.nanosec
            msg_time = int((msg_sec + msg_nsec * 1e-9) * 1000) 
            
            now_time = self.get_clock().now()
            sec, nsec = now_time.seconds_nanoseconds()
            self.msg_sub_time = int((sec + nsec * 1e-9) * 1000)


            print(f'image generated: {msg_time}')
            print(f'now_sub: {self.msg_sub_time}')
            print(f'node_start: {self.node_start_time}')

            image_timestamp = self.msg_sub_time  # milliseconds
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.current_image = cv_image
            self.frame_count += 1
 
            self.log_image_info(image_timestamp, self.node_start_time, msg_time, self.frame_count, self.is_recording, self.recording_timestamp)
            if self.is_recording and self.test_folder is not None:
                self.save_image(cv_image, msg_time)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def log_image_info(self, image_timestamp, node_start_time, img_gen_time, frame_count, is_recording, recording_timestamp):
        """Log image callback information to a text file in the test folder."""
        if self.test_base_folder is None:
            self.get_logger().debug('No test folder set up yet, skipping log entry')
            return
            
        try:
            log_file_path = os.path.join(self.test_base_folder, 'camera_log.txt')
            log_entry = f"Image Timestamp: {image_timestamp}, Node start Timestamp: {node_start_time}, Image gen Timestamp: {img_gen_time}\
                          Frame Count: {frame_count}, Is Recording: {is_recording}, Recording Timestamp: {recording_timestamp}\n"
            
            with open(log_file_path, 'a') as log_file:
                log_file.write(log_entry)
                
        except Exception as e:
            self.get_logger().error(f'Error writing to log file: {e}')

    def save_image(self, cv_image, image_timestamp):
        """Save image to the test folder."""
        print(f'test_num: {self.test_folder}')
        try:
            # Use the provided timestamp for consistent naming
            filename = f'frame_{image_timestamp}_{self.frame_count}.png'
            filepath = os.path.join(self.test_folder, filename)
            
            # Save image with success check
            success = cv2.imwrite(filepath, cv_image)
            if success:
                self.get_logger().debug(f'Saved image: {filename}')
            else:
                self.get_logger().error(f'Failed to save image: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'Error saving image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Interrupted, shutting down.')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
