# detection node written to sync crop info with image, mask them and send detection results
# author: Haosong Xiao
# testing date: 11/02/2025

# native imports
import os
import re
import sys

# pip installs
import cv2
import numpy as np

# ros2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from sensor_msgs.msg import Image
from std_msgs.msg import Int64MultiArray
import torch

# Add DeepCrack to path
sys.path.append("/inference/Deep_crack/")

from model.deepcrack import DeepCrack as DetectionModel
from trainer import DeepCrackTrainer as DetectionModelTrainer

# self written models, for results quantifications
# from quantification.YOLO_gt_HX import Yolo_gt_HX
from quantification.pixel_conversion import Cameras_HX

class Detection_node(Node):
    def __init__(self, weights_path='/inference/Deep_crack/DeepCrack_CT260_FT1.pth'):
        super().__init__('detection_node')

        init_time = self.get_clock().now()
        init_sec, init_nsec = init_time.seconds_nanoseconds()
        self.node_start_time = int((init_sec + init_nsec * 1e-9) * 1000)

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Using device: {self.device}')
        
        self.weights_path = weights_path
        self.model = self.load_model()
        self.get_logger().info('Model loaded successfully')
        
        # Declare parameters
        self.declare_parameter('normalization_mean', [114.0, 121.0, 134.0])

        # Subscribe to test number from on_vehicle_node
        self.test_num_sub = self.create_subscription(
            String, '/test_number', self.test_num_callback, 50)
        
        # Subscribe to recording indicator from on_vehicle_node
        self.process_sub = self.create_subscription(
            Int64MultiArray, '/ready_to_process', self.process_callback, 50)
        
        # Initialize ready_process and last processed timestamp
        self.ready_process = 0  
        self.last_processed_timestamp = None  

        # Timer to periodically check the ready_process flag
        self.create_timer(0.1, self.timer_callback) 

        self.get_logger().info('Waiting for process indicator to be ready...')

    def test_num_callback(self, msg: String):
        """Handle test number updates from on_vehicle_node."""
        self.test_num = msg.data
        self.setup_test_folder()

    def setup_test_folder(self):
        """Create test folder structure for image logging."""
        if self.test_num is None:
            return
            
        base = os.path.dirname(os.path.realpath(__file__))
        test_str = f"Test_{self.test_num}"
        self.test_base_folder = os.path.join(base, 'data', test_str)
        self.image_folder = os.path.join(self.test_base_folder, 'image_folder')
        self.detection_folder = os.path.join(self.test_base_folder, 'detection_folder') 
        os.makedirs(self.image_folder, exist_ok=True)
        os.makedirs(self.test_base_folder, exist_ok=True)
        os.makedirs(self.detection_folder, exist_ok=True)
        os.chmod(self.image_folder, 0o777)
        os.chmod(self.test_base_folder, 0o777)
        os.chmod(self.detection_folder, 0o777)

    def process_callback(self, msg: Int64MultiArray):
        """Handle processing state updates from on_vehicle_node."""
        if len(msg.data) >= 3:  
            self.ready_process = msg.data[2] 
            self.recording_timestamp = msg.data[0]  
            status = "started" if self.ready_process == 1 else "stopped"
            # self.get_logger().info(f'Recording {status}')
        else:
            pass
            # self.get_logger().warn(f'Recording message has insufficient data: {len(msg.data)} elements, expected at least 3')

    def load_model(self):
        '''Load the DeepCrack detection model'''
        model = DetectionModel().to(self.device)
        trainer = DetectionModelTrainer(model).to(self.device)
        
        try:
            state_dict = trainer.saver.load(self.weights_path, multi_gpu=False)
            model.load_state_dict(state_dict)
            model.eval()
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            raise
        return model
    
    @staticmethod
    def np2Tensor(array):
        """Convert numpy array to tensor"""
        if array.ndim == 2:  
            array = array[np.newaxis, ...]  
        elif array.ndim == 3:  
            array = array.transpose(2, 0, 1) 
        else:
            raise ValueError(f"Unsupported array shape: {array.shape}")
        return torch.from_numpy(array).float()
    
    def get_mask(self, image):
        """Generate segmentation mask from input image"""
        mean = np.array(self.get_parameter('normalization_mean').value,
                dtype=np.float32)
        img_norm = (image - mean) / 255.0
        input_tensor = self.np2Tensor(img_norm).unsqueeze(0).to(self.device)
        with torch.no_grad():
            pred = self.model(input_tensor)[0]  
            pred = torch.sigmoid(pred).cpu().squeeze().numpy() * 255
        mask = pred.astype(np.uint8)
        return mask
    
    def process_all(self):
     
        self.get_logger().info(f'ready to process: {self.ready_process}')
        if self.ready_process != 1:
            self.get_logger().info('Waiting for ready_process signal...')
            return

        crop_log = os.path.join(self.test_base_folder, 'detection_record_1_log.txt')
        raw_img_path = self.image_folder
        output_path = self.detection_folder

        self.get_logger().info('Step 1: Loading cropping coords...')
        
        detection_records = []
        with open(crop_log, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                record = re.search(r'record: (\d+),', line)
                if record and record.group(1) == '1':
                    timestamp_match = re.search(r'RTK gen_time:\s*(\d+)', line)
                    x1_match = re.search(r'x1:\s*([-\d]+)', line)
                    y1_match = re.search(r'y1:\s*([-\d]+)', line)
                    x2_match = re.search(r'x2:\s*([-\d]+)', line)
                    y2_match = re.search(r'y2:\s*([-\d]+)', line)
                    detection_records.append({
                        "timestamp": int(timestamp_match.group(1)) if timestamp_match else -1,
                        "x1": int(x1_match.group(1)) if x1_match else -1,
                        "y1": int(y1_match.group(1)) if y1_match else -1,
                        "x2": int(x2_match.group(1)) if x2_match else -1,
                        "y2": int(y2_match.group(1)) if y2_match else -1
                    })
        if not detection_records:
            self.get_logger().warn("No detection records found.")
            return

        self.get_logger().info(f'Loaded {len(detection_records)} croppable images')

        # Create output folders
        os.makedirs(output_path, exist_ok=True)
        crop_folder = os.path.join(output_path, 'crop')
        annotate_folder = os.path.join(output_path, 'annotate')
        mask_folder = os.path.join(output_path, 'mask')
        os.makedirs(crop_folder, exist_ok=True)
        os.makedirs(annotate_folder, exist_ok=True)
        os.makedirs(mask_folder, exist_ok=True)

        # SYNC images to the crop record
        image_files = sorted([f for f in os.listdir(raw_img_path) if f.endswith('.png')])
        if not image_files:
            self.get_logger().warn("No images found in image folder.")
            return

        self.get_logger().info(f'Step 2: Found {len(image_files)} croppable images to process')

        processed_count = 0
        for img_file in image_files:
            match = re.match(r'frame_(\d+)_(\d+)\.png', img_file)
            if not match:
                self.get_logger().warn(f"Skipping file with unexpected name format: {img_file}")
                continue

            img_ts = int(match.group(1))
            frame_count = int(match.group(2))
            raw_path = os.path.join(raw_img_path, img_file)
            raw_img = cv2.imread(raw_path)
            if raw_img is None:
                self.get_logger().warn(f"Failed to read image: {img_file}")
                continue

            # Find closest detection record by timestamp
            ts_diff = [abs(img_ts - det["timestamp"]) for det in detection_records]
            min_idx = ts_diff.index(min(ts_diff))
            det_info = detection_records[min_idx]

            x1, y1, x2, y2 = det_info["x1"], det_info["y1"], det_info["x2"], det_info["y2"]
            annotated_img = raw_img.copy()

            # log synced results:
            log_msg = f"Node start time: {self.node_start_time}, \
            synced crop time: {det_info['timestamp']}, \
            synced img time: {img_ts}\n"

            sync_record_log = os.path.join(self.test_base_folder, 'sync_report_log.txt')
            with open(sync_record_log, 'a') as f:
                    f.write(log_msg)



            if all(v != -1 for v in [x1, y1, x2, y2]):
                cropped_img = raw_img[y1:y2, x1:x2]
                crop_path = os.path.join(crop_folder, f'cropped_{img_ts}_{frame_count}.png')
                if not cv2.imwrite(crop_path, cropped_img):
                    self.get_logger().warn(f"Failed to save cropped image: {crop_path}")
                    continue

                mask = self.get_mask(cropped_img)
                mask_path = os.path.join(mask_folder, f'mask_{img_ts}_{frame_count}.png')
                cv2.imwrite(mask_path, mask)
                cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (255, 0, 0), 2)

            annotated_path = os.path.join(annotate_folder, f'annotated_{img_ts}_{frame_count}.png')
            cv2.imwrite(annotated_path, annotated_img)
            processed_count += 1

            self.last_processed_timestamp = img_ts  # [To-Do: I think we can use the latest process timestamp to reduce processing time]

            if processed_count % 10 == 0:
                self.get_logger().info(f'Processed {processed_count}/{len(image_files)} images')

        # self.get_logger().info(f'Processing complete: {processed_count} images processed')
        # self.get_logger().info(f'  - Cropped: {crop_folder}')
        # self.get_logger().info(f'  - Masks: {mask_folder}')
        # self.get_logger().info(f'  - Annotated: {annotate_folder}')


    def timer_callback(self):
        self.get_logger().info(f'ready to process: {self.ready_process}')
        if self.ready_process == 1:
            self.process_all()


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = Detection_node()
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
