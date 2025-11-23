import numpy as np
import os
import json 


script_dir = os.path.dirname(os.path.abspath(__file__))
config_path = os.path.join(script_dir, 'config', 'camera.json')

class Cameras_HX:
    def __init__(self, config_dir, yaw = 0, pitch = 0):
        self.config = self.load_config(config_dir)
        self.camera_intrinsics = self.config['camera_intrinsic']
        self.image_height = self.camera_intrinsics['image_height']
        self.image_width = self.camera_intrinsics['image_width']
        # Camera extrinsic: height, yaw, pitch, roll
        self.camera_extrinsic = self.config['camera_extrinsic']
        self.camera_height = self.camera_extrinsic['camera_height']
        # self.camera_yaw = self.camera_extrinsic['camera_yaw']
        # self.camera_pitch = self.camera_extrinsic['camera_pitch']
        self.camera_yaw = yaw
        self.camera_pitch = pitch
        self.camera_roll = self.camera_extrinsic['camera_roll']

        # GPS to camera offset in vehicle frame (x: forward, y: left, z: up)
        self.gps_to_camera_offset_body = self.config['gps_to_camera_offset_body']
        self.offset_x = self.gps_to_camera_offset_body['forward']
        self.offset_y = self.gps_to_camera_offset_body['left']
        self.offset_z = self.camera_height

    def load_config(self, config_dir):
        # Load configuration from a file (e.g., JSON, YAML)
        with open(config_dir, 'r') as f:
            config = json.load(f)
        return config

    def gps_to_local(self, crack_lat, crack_long, car_lat, car_long):
        """
        Convert GPS coordinates to local coordinate system relative to car antenna.
        
        Args:
            crack_gps: [lat, lon] of crack in degrees
            car_gps: [lat, lon] of car in degrees
            
        Returns:
            [x, y] in meters relative to car position (East, North)
        """
        # Convert to radians
        lat1, lon1 = np.radians(car_lat), np.radians(car_long)
        lat2, lon2 = np.radians(crack_lat), np.radians(crack_long)

        # earth radius has been pushed to config, for the sake of testing, I hard code it here
        R = 6371000

        # please note that the lat1 and long1 is the reference point
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        x = R * dlon * np.cos(lat1)  # East direction
        y = R * dlat                 # North direction
        
        return x, y


    def enu_to_car_frame(self, enu_point, car_heading):
        """
        Transform from ENU coordinates to car coordinate system
        
        Args:
            enu_point: [x, y] in ENU frame (East, North) 
            car_heading: heading angle in degrees from North (GPS convention)
            
        Returns:
            [x, y] in car frame (Forward, Left)
        """
        # GPS heading: 0° = North, 90° = East
        heading_rad = np.radians(car_heading)
        
        cos_h = np.cos(heading_rad)
        sin_h = np.sin(heading_rad)
        
        # this rotation gives crack w.r.t the car frame using car itself as origin
        R = np.array([[sin_h,  cos_h],
                    [-cos_h, sin_h]])
        
        return R @ enu_point


    def vehicle_to_standard_camera_frame(self):
        """
        Get the transformation matrix from vehicle frame to standard camera frame
        
        Vehicle Frame: X=Forward, Y=Left, Z=Up
        Standard Camera Frame: X=Right, Y=Down, Z=Forward
        
        Returns:
            3x3 transformation matrix
        """
        # Mapping from vehicle to camera:
        # Vehicle X (Forward) → Camera Z (Forward)
        # Vehicle Y (Left) → Camera -X   
        # Vehicle Z (Up) → Camera -Y 
        
        R_vehicle_to_standard_camera = np.array([
            [ 0, -1,  0],  # Camera X = -Vehicle Y 
            [ 0,  0, -1],  # Camera Y = -Vehicle Z 
            [ 1,  0,  0]   # Camera Z = Vehicle X 
        ])
        
        return R_vehicle_to_standard_camera


    def standard_camera_rotation_matrix(self):

        '''
        Combined rotation matrix for ZYX (yaw-pitch-roll) rotation sequence for camera frame. 

        Args:
            cam_yaw_deg: Yaw angle in degrees (rotation around Y-axis, left ccw positive)
            cam_pitch_deg: Pitch angle in degrees (rotation around X-axis, up ccw positive)
            cam_roll_deg: Roll angle in degrees (rotation around Z-axis, right ccw positive)

        Returns:
            3x3 rotation matrix considering camera extrinsic rotations
        '''
        yaw_rad = np.radians(self.camera_yaw)
        pitch_rad = np.radians(self.camera_pitch)
        roll_rad = np.radians(self.camera_roll)

        cos_r = np.cos(roll_rad)
        sin_r = np.sin(roll_rad)
        cos_p = np.cos(pitch_rad)
        sin_p = np.sin(pitch_rad)
        cos_y = np.cos(yaw_rad)
        sin_y = np.sin(yaw_rad)
        
        R_pitch = np.array([[1,     0,      0    ],
                        [0,  cos_p, -sin_p],
                        [0,  sin_p,  cos_p]])



        R_yaw = np.array([[ cos_y,  0,  sin_y],
                        [    0,   1,     0 ],
                        [-sin_y,  0,  cos_y]])



        R_roll = np.array([[cos_r, -sin_r,  0],
                        [sin_r,  cos_r,  0],
                        [   0,      0,   1]])

        R_zyx = R_roll @ R_pitch @ R_yaw
        return R_zyx


    def vehicle_to_camera_frame(self, crack_position_vehicle):
        """
        Transform from vehicle coordinate system to camera coordinate system.
    
        Args:
            crack_position_vehicle: [x, y] or [x, y, z] in vehicle frame (Forward, Left, Up)
            camera_offset: [x, y, z] camera position relative to antenna in vehicle frame
            camera_orientation: [yaw, pitch, roll] camera orientation in degrees.
                            - yaw: Rotation around the camera's Y-axis (Down).
                            - pitch: Rotation around the camera's X-axis (Right).
                            - roll: Rotation around the camera's Z-axis (Forward/Optical).
            
        Returns:
            [x, y, z] in camera frame (Right, Down, Forward/Optical axis)
        """
        # assuming crack is on ground
        if len(crack_position_vehicle) == 2:
            crack_3d = np.array([crack_position_vehicle[0], crack_position_vehicle[1], 0.0])
        else:
            crack_3d = np.array(crack_position_vehicle)
        
        # Step 1
        # print("Crack in vehicle frame (Forward, Left, Up):", crack_3d)
        crack_relative_to_camera = crack_3d - np.array([self.offset_x, self.offset_y, self.offset_z])
        # print("Crack relative to camera (Vehicle frame):", crack_relative_to_camera)
        # Step 2: Transform to standard camera orientation (Vehicle → Standard Camera)
        R_vehicle_to_standard = self.vehicle_to_standard_camera_frame()
        crack_in_standard_camera = R_vehicle_to_standard @ crack_relative_to_camera
        
        # Step 3: Apply camera's intrinsic yaw, pitch, roll rotations
        R_camera_rotation = self.standard_camera_rotation_matrix()

        # Final transformation
        crack_in_camera_frame = R_camera_rotation @ crack_in_standard_camera
        
        return crack_in_camera_frame


    def project_to_pixel(self, point_3d_camera):
        """
        Project a 3D point in camera coordinates to 2D pixel coordinates.
        
        Args:
            point_3d_camera: [X, Y, Z] in camera frame (Right, Down, Forward).
            camera_intrinsics: Dictionary with 'fx', 'fy', 'cx', 'cy'.
            
        Returns:
            (u, v) pixel coordinates, or None if point is behind camera.
        """
        X, Y, Z = point_3d_camera
        fx, fy = self.camera_intrinsics['fx'], self.camera_intrinsics['fy']
        cx, cy = self.camera_intrinsics['cx'], self.camera_intrinsics['cy']

        # Point must be in front of the camera (Z > 0)
        if Z <= 0:
            return -1, -1
            
        # Projection formulas
        u = fx * (X / Z) + cx
        v = fy * (Y / Z) + cy

        return int(u), int(v)


    def pixel_to_vehicle_frame(self, pixel_coords):
        """
        Back-project a 2D pixel coordinate to a 3D point in the vehicle frame,
        assuming the point lies on the ground plane (Z_vehicle = 0).
        
        Args:
            pixel_coords: (u, v) pixel coordinates.
            camera_intrinsics: Dictionary with 'fx', 'fy', 'cx', 'cy'.
            camera_offset: [x, y, z] camera position in vehicle frame.
            camera_orientation: [yaw, pitch, roll] camera orientation in degrees,
                            defined as intrinsic rotations (see vehicle_to_camera_frame).
            
        Returns:
            3D point [X, Y, Z] in vehicle frame, or None if no solution.
        """
        u, v = pixel_coords
        fx, fy = self.camera_intrinsics['fx'], self.camera_intrinsics['fy']
        cx, cy = self.camera_intrinsics['cx'], self.camera_intrinsics['cy']

        # Step 1: Un-project pixel to a normalized 3D ray in the camera frame
        # This ray represents all possible 3D points that could project to (u,v)
        # The point is [X, Y, Z] = Z * [ (u-cx)/fx, (v-cy)/fy, 1 ]
        normalized_ray_camera = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        
        # Step 2: Get the inverse rotation matrices
        R_camera_rotation = self.standard_camera_rotation_matrix()
        # Inverse rotation: Rotated Camera -> Standard Camera
        R_inv_camera_rotation = R_camera_rotation.T
        
        # Forward transform: Vehicle -> Standard Camera
        R_vehicle_to_standard = self.vehicle_to_standard_camera_frame()
        # Inverse transform: Standard Camera -> Vehicle
        R_standard_to_vehicle = R_vehicle_to_standard.T
        
        # Step 3: Transform the ray from camera frame back to vehicle frame
        # First, from rotated camera to standard camera
        ray_standard_camera = R_inv_camera_rotation @ normalized_ray_camera
        # Then, from standard camera to vehicle
        ray_vehicle = R_standard_to_vehicle @ ray_standard_camera
        
        # Step 4: Solve for the intersection with the ground plane (Z_vehicle = 0)    
        cam_offset_vec = np.array([self.offset_x, self.offset_y, self.offset_z])
        # Avoid division by zero if the ray is parallel to the ground plane
        if np.abs(ray_vehicle[2]) < 1e-6:
            return None # No unique intersection
            
        # Solve for d
        d = -cam_offset_vec[2] / ray_vehicle[2]
        
        # If d is negative, the intersection is behind the camera, which is not valid
        if d < 0:
            return None
            
        # Step 5: Calculate the final 3D point in the vehicle frame
        point_vehicle = cam_offset_vec + d * ray_vehicle
        
        return point_vehicle


