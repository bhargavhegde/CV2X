#!/usr/bin/env python3

'''
This file contains the main classes that process the raw images with cropping windows
'''


# Standard library imports
import os
import sys
import time
from typing import Optional

# Third-party imports
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import torch

# Local imports
current_dir = os.path.dirname(__file__)
sys.path.append(os.path.join("/Deep_crack_inference"))
from model.deepcrack import DeepCrack 
from trainer import DeepCrackTrainer
model_name = "Deep Crack"



class ImageProcessor(Node):
    def __init__(self, shared_queue):
        super().__init__('image_processor')
        self.get_logger().info('Starting ImageProcessor…')

        self.curr_count = 0 # TODO: check if this is still needed. the msg definition changed.

        self.declare_parameter('queue_size', 10000)
        self.declare_parameter('crop_coords', [1032, 1544, 850, 1362])   # y1,y2,x1,x2
        self.declare_parameter('normalization_mean', [114., 121., 134.])
        self.declare_parameter('display_images', False)
        self.declare_parameter('camera_topic_prefix', '/vimbax_camera_')
        self.declare_parameter('camera_topic_suffix', '/image_raw')
        # [CRH] this pth exist in the docker container
        self.declare_parameter('deepcrack_weights',
                               '/Deep_crack_inference/DeepCrack_CT260_FT1.pth')    
        self.declare_parameter('crop_coord_topic', '/crack_pixel') 
        crop_coord_topic = self.get_parameter('crop_coord_topic').value
        self.crop_coord = None 

        # Communication Parameters
        # [CRH] every time there is a new image, it is added to the queue.
        self.create_timer(0.1, self.push_queue)
        self.q = shared_queue
        self.curr_image = None

        q = self.get_parameter('queue_size').value
        prefix = self.get_parameter('camera_topic_prefix').value
        suffix = self.get_parameter('camera_topic_suffix').value

        self.bridge = CvBridge()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Using device: {self.device}')

        try:
            self.model= self.load_model()
            self.get_logger().info(f'{model_name} loaded ✓')
        except Exception as e:
            self.get_logger().error(f'Could not load {model_name}: {e}')
            raise


        input_topic = self.find_image_topic(prefix, suffix)
        if input_topic is None:
            raise RuntimeError('Camera topic not found')

        # [CRH] this will be called whenever there is a new image
        self.subscription = self.create_subscription(Image,
                                                     input_topic,
                                                     self.image_callback,
                                                     q)
        # [TODO] old version without GPS transmitted.
        # self.crop_coord_sub = self.create_subscription(Int32MultiArray, crop_coord_topic, self.crop_center_callback, 10)
        self.crop_coord_sub = self.create_subscription(Float32MultiArray, crop_coord_topic, self.crop_center_callback, 10)
        self.get_logger().info(f'Subscribed to {input_topic} and {crop_coord_topic}')


        base = os.getcwd()
        timestamp = time.strftime('%Y%m%d%H%M%S')
        # timestamp = str(time.time()).replace('.', '_')  # Use timestamp with seconds
        self.output_dirs = {k: os.path.join(base,'data', timestamp, k)
                            for k in ['cropped_images','images', 'masks', 'latest']} #+ timestamp
        for p in self.output_dirs.values():
            os.makedirs(p, exist_ok=True)
            os.chmod(p, 0o777)


        self.frame_count = 0
        self.processing_times = []

    def crop_center_callback(self, msg: Float32MultiArray):
        self.crop_coord = msg.data
        self.get_logger().debug(f'Updated crop center to {self.crop_coord}')

    def find_image_topic(self, prefix: str, suffix: str) -> Optional[str]:
        for topic, types in self.get_topic_names_and_types():
            if topic.startswith(prefix) and topic.endswith(suffix) and \
               'sensor_msgs/msg/Image' in types:
                return topic
        return None

    def load_model(self):
        """Instantiate DeepCrack + trainer, load weights, set eval mode."""
        weights = self.get_parameter('deepcrack_weights').value

        model = DeepCrack().to(self.device)       
        trainer = DeepCrackTrainer(model).to(self.device)

        state_dict = trainer.saver.load(weights, multi_gpu=False)
        model.load_state_dict(state_dict)       

        model.eval()
        return model
    
    def push_queue(self):
        if self.curr_image is not None:
            self.q.put(self.curr_image)

    @staticmethod
    def np2Tensor(array: np.ndarray) -> torch.Tensor:
        """HWC/BGR‒>CHW tensor float32."""
        if array.ndim == 2:            
            array = array[np.newaxis]
        else:                           # HWC
            array = array.transpose(2, 0, 1)
        return torch.from_numpy(array).float()

    def image_callback(self, msg: Image):
        '''
            [CRH] when no new image is received, it stuck with the old one?
            TODO need to do some image preprocessing here.
        '''
        tic = time.time()
        self.frame_count += 1
        try:
            
            if self.crop_coord is None:
                self.get_logger().warn('No crop center received yet, skipping frame. Still save full image.')
                # still save the full image
                # TODO: can skip this if not saving the full image
                # return
            
            cv_img_full = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width = cv_img_full.shape[:2]

            # check if crop coordinate msg is valid
            if not self.crop_coord or len(self.crop_coord) != 7:
                self.get_logger().warn('Invalid crop coordinates received, skipping frame. Still save full image.')
                valid_crop = False
            else:
                x1,y1,x2,y2, gps_lat, gps_lon, gps_heading = self.crop_coord
                valid_crop = True

            if not valid_crop or not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
                self.get_logger().warn('Crop coordinates out of bounds, skipping frame. Still save full image.')
                # still save the full image
                # TODO: note that this time may not be the same as the actual frame taken time
                ts = int(time.time())
                paths = {
                'img': os.path.join(self.output_dirs['images'],
                                    f'frame_{ts}_{self.frame_count}.png'),
                }
                cv2.imwrite(paths['img'], cv_img_full)
                return

            # Crop the image
            cv_img = cv_img_full[y1:y2, x1:x2]
            cv2.rectangle(cv_img_full, (x1, y1), (x2, y2), (255, 0, 0), 2)

            mean = np.array(self.get_parameter('normalization_mean').value,
                            dtype=np.float32)
            img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB).astype(np.float32)
            img_norm = (img_rgb - mean) / 255.0
            input_tensor = self.np2Tensor(img_norm).unsqueeze(0).to(self.device)

            # Run inference
            # TODO: speed this up with real-time inference e.g. with ONNX or TensorRT
            with torch.no_grad():
                pred = self.model(input_tensor)[0]  # single forward
                pred = torch.sigmoid(pred).cpu().squeeze().numpy() * 255
            mask = pred.astype(np.uint8)

            ts = int(time.time())
            paths = {
                'img': os.path.join(self.output_dirs['images'],
                                    f'frame_{ts}_{self.frame_count}.png'),
                'crop':os.path.join(self.output_dirs['cropped_images'],
                                    f'frame_{ts}_{self.frame_count}.png'),
                'mask': os.path.join(self.output_dirs['masks'],
                                     f'frame_{ts}_{self.frame_count}.png'),
                'latest_img': os.path.join(self.output_dirs['latest'],
                                           'latest_image_frame.jpg'),
                'latest_mask': os.path.join(self.output_dirs['latest'],
                                            'latest_mask_frame.jpg'),
                'latest_txt': os.path.join(self.output_dirs['latest'],
                                           'latest_time.txt')
            }
            image_name = f'frame_{ts}_{self.frame_count}.png'
            self.curr_image = str(os.path.basename(paths['mask']))

            # cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            cv2.imwrite(paths['crop'], cv_img)
            cv2.imwrite(paths['img'], cv_img_full)
            cv2.imwrite(paths['mask'], mask)
            
            # additionally save the latest image and mask
            # cv2.imwrite(paths['latest_img'], cv_img)
            # cv2.imwrite(paths['latest_mask'], mask)
            # with open(paths['latest_txt'], 'w') as f:
            #     f.write(f'{ts}_{self.frame_count}')

            # Logging Relevant Information
            image_log = f"Image_Name: {image_name}, Lat: {gps_lat}, Lon: {gps_lon}, heading: {gps_heading}, x1: {x1}, x2: {x2}, y1: {y1}, y2: {y2}\n"
            image_log_filename = 'realtime_data_log.txt'

            with open(image_log_filename, "a") as log_file:
                log_file.write(image_log)


            if self.get_parameter('display_images').value:
                cv2.imshow('DeepCrack – input', cv_img)
                cv2.imshow('DeepCrack – mask', mask)
                cv2.waitKey(1)

            self.processing_times.append(time.time() - tic)
            if self.frame_count % 10 == 0:
                avg = np.mean(self.processing_times[-10:])
                self.get_logger().info(f'{self.frame_count} frames | '
                                       f'avg {avg:.3f}s | '
                                       f'curr {self.processing_times[-1]:.3f}s')
        except Exception as e:
            self.get_logger().error(f'Frame {self.frame_count} failed: {e}')


# def main(args=None):
#     rclpy.init(args=args)
#     node = None
#     try:
#         node = ImageProcessor()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         if node:
#             node.get_logger().info('Interrupted, shutting down.')
#     finally:
#         if node:
#             node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

def save_images(shared_queue):
    # rclpy.init(args=args)
    node = None
    try:
        node = ImageProcessor(shared_queue)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Interrupted, shutting down.')
    finally:
        if node:
            node.destroy_node()
