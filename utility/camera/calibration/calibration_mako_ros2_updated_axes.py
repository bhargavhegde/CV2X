import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
from scipy.spatial.transform import Rotation as R

# ---------------------- CONFIG ----------------------
CHECKERBOARD = (6, 9)
square_size = 6.95  # updated for new checkerboard (in cm)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp[:, :2] *= square_size

# Choose mode: 'full_calibration' or 'json_intrinsics'
MODE = 'json_intrinsics'
INTRINSICS_FILE = "camera.json"
SAVE_PARAMS_FILE = "mako_parameters_updated.json"

# ---------------------- ROS2 Node ----------------------
class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.image = None
        self.subscription = self.create_subscription(
            Image,
            '/vimbax_camera_74/image_raw',  # Ensure this matches your ROS2 topic
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image = cv_image
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")

# ---------------------- HELPER FUNCTIONS ----------------------
def capture_checkerboard_image(node):
    print("Waiting for checkerboard image...")
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.image is not None:
            # Automatically capture the first available frame
            img = node.image.copy()
            cv2.imwrite("checkerboard_capture.jpg", img)  # Optional: save for verification
            print("Captured image for extrinsics as 'checkerboard_capture.jpg'")
            return img

def find_corners(gray_image):
    found, corners = cv2.findChessboardCorners(gray_image, CHECKERBOARD, None)
    if found:
        corners2 = cv2.cornerSubPix(gray_image, corners, (11,11), (-1,-1), criteria)
        return True, corners2
    return False, None

# ---------------------- MAIN LOGIC ----------------------
def main():
    rclpy.init()
    cam_node = CameraNode()

    saved_params = {}

    if MODE == 'full_calibration':
        objpoints = []
        imgpoints = []
        print("Starting full calibration. Press 'c' to capture each checkerboard image.")
        while len(objpoints) < 5 and rclpy.ok():
            rclpy.spin_once(cam_node, timeout_sec=0.1)
            if cam_node.image is None:
                continue
            frame = cam_node.image.copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners2 = find_corners(gray)
            display = frame.copy()
            if found:
                cv2.drawChessboardCorners(display, CHECKERBOARD, corners2, found)
            cv2.imshow("Calibration Feed", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c') and found:
                objpoints.append(objp.copy())
                imgpoints.append(corners2)
                print(f"Captured image #{len(objpoints)}")
            elif key == ord('q'):
                break
        cv2.destroyAllWindows()

        if len(objpoints) < 5:
            print("Not enough images for calibration.")
            cam_node.destroy_node()
            rclpy.shutdown()
            return

        # Camera calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )
        print("Calibration done. Camera matrix:\n", mtx)
        print("Distortion coefficients:\n", dist)
        saved_params['camera_matrix'] = mtx.tolist()
        saved_params['distortion_coefficients'] = dist.tolist()

    elif MODE == 'json_intrinsics':
        print("Loading camera intrinsics from JSON.")
        with open(INTRINSICS_FILE, "r") as f:
            camera_data = json.load(f)
        fx = camera_data["camera_intrinsic"]["f_x"]
        fy = camera_data["camera_intrinsic"]["f_y"]
        cx = camera_data["camera_intrinsic"]["c_x"]
        cy = camera_data["camera_intrinsic"]["c_y"]
        mtx = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]], dtype=np.float64)
        dist = np.zeros((5,1), dtype=np.float64)
        print("Camera matrix:\n", mtx)
        saved_params['camera_matrix'] = mtx.tolist()
        saved_params['distortion_coefficients'] = dist.tolist()

    # Extrinsic calculation
    img = capture_checkerboard_image(cam_node)
    if img is None:
        print("No image captured. Exiting.")
        cam_node.destroy_node()
        rclpy.shutdown()
        return

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners2 = find_corners(gray)
    if not found:
        print("Checkerboard not found in image. Exiting.")
        cam_node.destroy_node()
        rclpy.shutdown()
        return

    retval, rvec, tvec = cv2.solvePnP(objp, corners2, mtx, dist)

    R_wc, _ = cv2.Rodrigues(rvec)

    # Transform the extrinsic matrix to new coordinate system
    T = np.array([
        [0,  0, 1],
        [-1, 0, 0],
        [0, -1, 0]
    ])
    R_new = T @ R_wc
    t_new = T @ tvec
    extrinsic_new = np.hstack((R_new, t_new))

    # Keep original yaw-pitch-roll (based on OpenCV coordinate frame)
    yaw, pitch, roll = R.from_matrix(R_wc).as_euler('yxz', degrees=True)

    print("Transformed Extrinsic matrix [R|t]:\n", extrinsic_new)
    print(f"Yaw={yaw:.2f}°, Pitch={pitch:.2f}°, Roll={roll:.2f}°")

    cv2.imwrite("extrinsic_image.jpg", img)
    saved_params.update({
        "translation_vector": t_new.tolist(),
        "transformed_extrinsic_matrix": extrinsic_new.tolist(),
        "yaw_pitch_roll_degrees": [float(yaw), float(pitch), float(roll)]
    })



    with open(SAVE_PARAMS_FILE, "w") as f:
        json.dump(saved_params, f, indent=4)
    print(f"All parameters saved to {SAVE_PARAMS_FILE}")

    cam_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()