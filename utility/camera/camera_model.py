'''
    # camera_model.py
    A simple camera projection model, intended for a fixed mounted, vehicle forward facing camera.
    Written with the help of Copilot. (Verified)

    The model takes in intrinsic parameters (focal lengths, principal point, skew) as inherit parameters,
    Inputs:
        - camera locations relative to the vehicle antenna
        - the camera orientation relative to the vehicle body (yaw, pitch, roll)
        - the vehicle heading and ENU position of the GPS antenna
        - a list of world points with ENU coordinates
    Outputs:
        - the camera coordinates of the world points
        - the projected image pixel coordinates of the world points
        - a visualization of the coordinate systems and projections

    Major assumptions:
    - Camera is rigidly mounted on vehicle, with known offset from GPS antenna
    - Vehicle coordinate system: x-forward, y-left, z-up, vehicle heading=0 is east, 90 is north, etc.
    - Vehicle sticks to the ground plane (no suspension effects), so no roll/pitch from vehicle motion
    - Camera coordinate system: x-right, y-down, z-forward (when yaw=pitch=roll=0)
    - World coordinate system: ENU (x-east, y-north, z-up)
'''
import argparse
import logging
import os

import matplotlib.pyplot as plt
import numpy as np

# Configure module-level logger
logger = logging.getLogger("camera_model")
logger.setLevel(logging.INFO)  # Default level, can be changed by main script
if not logger.handlers:  # Avoid duplicate handlers
    handler = logging.StreamHandler()
    formatter = logging.Formatter(
        '%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)d] - %(message)s'
    )
    handler.setFormatter(formatter)
    logger.addHandler(handler)


class CameraModel:
    def __init__(self, gps_to_cam_offset_body, intrinsics):
        # measured in body frame.
        self.gps_to_cam_offset_body = gps_to_cam_offset_body
        self.intrinsics = intrinsics
        # columns are body frame in camera frame
        # so p_c = self.R_body_to_cam * p_b
        self.R_body_to_cam = np.array([
            # from body (x - forward, y - left, z - up) to cam (x - right, y - down, z - forward)
            [0, -1, 0],
            [0, 0, -1],
            [1, 0, 0]
        ])
        # Quick verification, first roll 90, then pitch -90, no yaw
        yaw, pitch, roll = 0, 90, -90
        # note that this matrix is used to convert from coordinate in the new frame to the coordinate in the old frame
        # p_old = R @ p_new
        # thus transpose is needed to convert from old to new, consistent with the definition of R_body_to_cam
        R_body_to_cam_verify = self.rotation_matrix_zyx(yaw, pitch, roll).T

        logger.debug(f"R_body_to_cam:\n{self.R_body_to_cam}")
        logger.debug(f"R_body_to_cam_verify:\n{R_body_to_cam_verify}")
        assert np.allclose(self.R_body_to_cam, R_body_to_cam_verify), \
            "R_body_to_cam does not match expected rotation"
        # a trivial verification
        yaw_rand = np.random.uniform(-180, 180)
        assert np.allclose(
            self.rotation_matrix_yaw(yaw_rand),
            self.rotation_matrix_zyx(yaw_rand, 0, 0)
        ), "Yaw rotation matrix does not match expected for zero rotation"

    def rotation_matrix_yaw(self, heading_deg):
        '''
        heading is relative to x-axis (east if it is ENU)
        positive is counter-clockwise
        the result are columns of the new frame (after rotation) in the old frame (before rotation)
        i.e. R @ v_new = v_old
        '''
        heading_rad = np.deg2rad(heading_deg)
        cos_h = np.cos(heading_rad)
        sin_h = np.sin(heading_rad)
        return np.array([
            [cos_h, -sin_h, 0],
            [sin_h, cos_h, 0],
            [0, 0, 1]
        ])

    def rotation_matrix_zyx(self, yaw, pitch, roll):
        '''
            yaw, pitch, roll in degrees
            yaw is always about z
            pitch is always about y
            roll is always about x
            and rotation matrix are not commutative
            so order matters
            It has to be roll first, then pitch, then yaw
            The result are columns of the new frame (after rotation) in the old frame (before rotation)
            i.e. R @ v_new = v_old
        '''
        yaw = np.deg2rad(yaw)
        pitch = np.deg2rad(pitch)
        roll = np.deg2rad(roll)

        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        # logger.debug(f"Rz:\n{Rz}\nRy:\n{Ry}\nRx:\n{Rx}")
        # extrinsic rotation matrix from body to camera frame
        # careful with the order
        # https://en.wikipedia.org/wiki/Euler_angles#Tait%E2%80%93Bryan_angles
        return Rx @ Ry @ Rz

    def world_to_camera(self, world_point, antenna_pos_world, heading_deg,
                        yaw, pitch, roll):
        '''
        Main function to convert world coordinates to camera coordinates.
        inputs !!!Important!!!:
                world_point: 3D point in world coordinates (ENU)
            antenna_pos_world: GPS antenna position in world coordinates (ENU)
                Note that this value does not need to be the absolute position;
                for example, it can be [0,0,0], in which case the world point
                returned will be relative to this position but still in the world frame (ENU).
            heading_deg: vehicle heading angle in degrees (0 = East, 90 = North, etc.)
                camera orientation angles in vehicle body frame:
            yaw (Azimuth): rotation around body frame Z axis (up), positive is counter-clockwise or to the left
            pitch (Attitude): rotation around body frame X axis (right), positive is counter-clockwise
            roll (Bank): rotation around body frame Y axis (down)
        '''
        R_heading = self.rotation_matrix_yaw(heading_deg)
        offset_world = R_heading @ self.gps_to_cam_offset_body
        camera_origin_world = antenna_pos_world + offset_world
        rel_point = world_point - camera_origin_world
        R_cam = self.rotation_matrix_zyx(yaw, pitch, roll)
        # R_heading.T converts world coordinate to body frame
        # R_cam.T converts to camera frame but using forward-x, left-y and up-z
        # R_body_to_cam converts body frame to camera frame (right-x, down-y, front-z)
        R_total = self.R_body_to_cam @ R_cam.T @ R_heading.T
        camera_coords = R_total @ rel_point
        # This will be a point in camera frame, p_c ready to do image projection
        return camera_coords

    def project_point_to_image(self, point_cam):
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        cx = self.intrinsics['cx']
        cy = self.intrinsics['cy']
        gamma = self.intrinsics.get('gamma', 0.0)

        K = np.array([
            [fx, gamma, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])

        x, y, z = point_cam
        # logger.debug(f"Projecting point {point_cam} to image coordinates")
        if z <= 0:
            return None  # Point behind the camera

        p_cam = np.array([x, y, z])
        p_img_homog = K @ (p_cam / z)
        # logger.debug(f"Projected image coordinates (homogeneous): {p_img_homog}")
        return p_img_homog[:2]

    def camera_pixel_to_world(self, pixel_uv,
                              antenna_pos_world, heading_deg, yaw, pitch,
                              roll):
        """
        Convert camera pixel coordinates to world coordinates on the ground plane.
        This function uses vehicle antenna position and camera relative position to calculate the world coordinates.
        TODO: have a projection that uses solely the camera position/orientation and the pixel coordinates.
        Args:
            pixel_uv: [u,v] pixel coordinates in the image
            antenna_pos_world: GPS antenna position in world coordinates (ENU).
                Note that this value does not need to be the absolute position;
                for example, it can be [0,0,0], in which case the world point
                returned will be relative to this position but still in the world frame (ENU).
            heading_deg: vehicle heading in degrees (0=East, 90=North)
            yaw, pitch, roll: camera orientation angles in vehicle body frame

        Returns:
            world_point: corresponding point in world coordinates (ENU)
        """
        # 1. Convert pixel to normalized camera coordinates
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        cx = self.intrinsics['cx']
        cy = self.intrinsics['cy']
        # gamma = self.intrinsics.get('gamma', 0.0)

        u, v = pixel_uv
        x_norm = (u - cx) / fx
        y_norm = (v - cy) / fy
        ray_dir_cam = np.array([x_norm, y_norm, 1.0])  # Camera frame ray direction

        # 2. Transform ray to world coordinates
        R_heading = self.rotation_matrix_yaw(heading_deg)
        R_cam = self.rotation_matrix_zyx(yaw, pitch, roll)
        R_total = R_heading @ R_cam @ self.R_body_to_cam.T
        ray_dir_world = R_total @ ray_dir_cam

        # 3. Get camera position in world coordinates
        # antenna offset has height so no need consider height separately here.
        cam_pos_world = antenna_pos_world + R_heading @ self.gps_to_cam_offset_body

        # 4. Find intersection with ground plane (z = 0)
        # ray equation: point = cam_pos + t * ray_dir
        # ground plane equation: z = 0
        # solve: cam_pos[2] + t * ray_dir[2] = 0
        if abs(ray_dir_world[2]) < 1e-6:
            return None  # Ray is parallel to ground

        t = -cam_pos_world[2] / ray_dir_world[2]
        if t <= 0:
            return None  # Point is behind camera

        world_point = cam_pos_world + t * ray_dir_world
        return world_point

    # Utility function for visualization.
    def plot_coordinate_systems(self, antenna_pos_world, heading_deg, yaw, pitch, roll):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot GPS antenna location
        ax.scatter(*antenna_pos_world, color='blue', label='GPS Antenna')

        # Get camera origin in world coordinates
        R_heading = self.rotation_matrix_yaw(heading_deg)
        cam_origin_world = antenna_pos_world + R_heading @ self.gps_to_cam_offset_body
        ax.scatter(*cam_origin_world, color='green', label='Camera Origin')

        # Plot vehicle body frame at antenna (x-forward, y-left, z-up)
        axis_length = 1.0
        # Transform body frame axes to world frame
        body_x = R_heading @ np.array([axis_length, 0, 0])
        body_y = R_heading @ np.array([0, axis_length, 0])
        body_z = R_heading @ np.array([0, 0, axis_length])

        ax.quiver(*antenna_pos_world, *body_x, color='red', linewidth=2,
                  label='Body X (Forward)')
        ax.quiver(*antenna_pos_world, *body_y, color='green', linewidth=2,
                  label='Body Y (Left)')
        ax.quiver(*antenna_pos_world, *body_z, color='blue', linewidth=2,
                  label='Body Z (Up)')

        # Plot camera frame at camera origin (x-right, y-down, z-forward)

        R_cam = self.rotation_matrix_zyx(yaw, pitch, roll)

        R_total = R_heading @ R_cam @ self.R_body_to_cam.T

        cam_x = R_total @ np.array([axis_length, 0, 0])
        cam_y = R_total @ np.array([0, axis_length, 0])
        cam_z = R_total @ np.array([0, 0, axis_length])

        ax.quiver(*cam_origin_world, *cam_x, color='red', linestyle='--',
                  linewidth=2, label='Cam X (Right)')
        ax.quiver(*cam_origin_world, *cam_y, color='green', linestyle='--',
                  linewidth=2, label='Cam Y (Down)')
        ax.quiver(*cam_origin_world, *cam_z, color='blue', linestyle='--',
                  linewidth=2, label='Cam Z (Forward)')

        # Set axis limits with buffer
        axis_range = 2.0 * axis_length  # Adjust this value to control visualization size
        center = np.mean([antenna_pos_world, cam_origin_world], axis=0)
        ax.set_xlim(center[0] - axis_range, center[0] + axis_range)
        ax.set_ylim(center[1] - axis_range, center[1] + axis_range)
        ax.set_zlim(center[2] - axis_range, center[2] + axis_range)

        ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio
        # ax.set_aspect('equal')  # Make axes equal scale
        ax.set_xlabel('East')
        ax.set_ylabel('North')
        ax.set_zlabel('Up')
        ax.set_title('Body and Camera Coordinate Frames')
        ax.legend()
        plt.tight_layout()
        return fig, ax

    def get_camera_fov_polygon(self, antenna_pos_world, heading_deg,
                               yaw, pitch, roll,
                               max_distance=50.0, ground_height=0.0):
        """
        Calculate the camera's field of view as a polygon in world coordinates.

        Args:
            antenna_pos_world: GPS antenna position in world coordinates (ENU)
            heading_deg: vehicle heading angle in degrees
            yaw, pitch, roll: camera orientation angles in vehicle body frame
            max_distance: maximum distance to project the FOV (meters)
            ground_height: height of the ground plane (meters)

        Returns:
            fov_polygon: array of world coordinates defining the FOV boundary
        """
        # Get camera intrinsics
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        cx = self.intrinsics['cx']
        cy = self.intrinsics['cy']

        # Calculate image dimensions (assuming image center is at cx, cy)
        img_width = cx * 2
        img_height = cy * 2

        # Define image corners in pixel coordinates
        image_corners = np.array([
            [0, 0],                    # top - left
            [img_width, 0],            # top - right
            [img_width, img_height],   # bottom - right
            [0, img_height]            # bottom - left
        ])

        # Convert image corners to camera rays (normalized direction vectors)
        camera_rays = []
        for u, v in image_corners:
            # Convert pixel to normalized camera coordinates
            x_norm = (u - cx) / fx
            y_norm = (v - cy) / fy
            z_norm = 1.0  # Forward direction

            # Normalize the ray direction
            ray_dir = np.array([x_norm, y_norm, z_norm])
            ray_dir = ray_dir / np.linalg.norm(ray_dir)
            camera_rays.append(ray_dir)

        # Compute camera pose in world coordinates
        R_heading = self.rotation_matrix_yaw(heading_deg)
        cam_origin_world = antenna_pos_world + R_heading @ self.gps_to_cam_offset_body

        # Transform camera rays to world coordinates
        R_cam = self.rotation_matrix_zyx(yaw, pitch, roll)
        R_total = R_heading @ R_cam @ self.R_body_to_cam.T

        # Calculate intersection points with ground plane or max distance
        # fov_points = [cam_origin_world]  # Start with camera origin
        fov_points = [] # lower edge is and top edge
        for ray_cam in camera_rays:
            # Transform ray to world coordinates
            ray_world = R_total @ ray_cam

            # Calculate intersection with ground plane (z = ground_height)
            # ray equation: point = cam_origin + t * ray_world
            # ground plane: z = ground_height
            # solve: cam_origin[2] + t * ray_world[2] = ground_height

            # use 4 edges of the image to form the FOV polygon

            if abs(ray_world[2]) > 1e-6:  # Ray is not parallel to ground
                t_ground = (ground_height - cam_origin_world[2]) / ray_world[2]
                if t_ground > 0:  # Ray goes toward ground
                    ground_point = cam_origin_world + t_ground * ray_world
                    distance = np.linalg.norm(ground_point[:2] - cam_origin_world[:2])
                    logger.debug(f"Ray {ray_cam} intersects ground at {ground_point} with distance {distance:.2f}m")

                    if distance <= max_distance:
                        fov_points.append(ground_point)
                    else:
                        # Use max distance instead
                        t_max = max_distance / np.linalg.norm(ray_world[:2])
                        max_point = cam_origin_world + t_max * ray_world
                        fov_points.append(max_point)
                else:
                    # Ray goes away from ground, use max distance
                    t_max = max_distance / np.linalg.norm(ray_world)
                    max_point = cam_origin_world + t_max * ray_world
                    fov_points.append(max_point)
            else:
                # Ray is parallel to ground, use max distance
                t_max = max_distance / np.linalg.norm(ray_world)
                max_point = cam_origin_world + t_max * ray_world
                fov_points.append(max_point)

        # Close the polygon by returning to the first corner
        fov_points.append(fov_points[0])
        logger.debug(f"FOV points:\n{np.array(fov_points)}")
        return np.array(fov_points)

    def plot_comparison_view(self, world_points, antenna_pos_world,
                             heading_deg, yaw, pitch, roll,
                             roi_length=1.0, roi_width=0.8):
        '''
        same input as world_to_camera method
        TODO: refactor to breakdown into smaller functions, and see if we can reuse some code
        '''
        fig, (ax_top, ax_img) = plt.subplots(1, 2, figsize=(14, 6))

        # Ensure world_points is a list/array of points
        world_points = np.atleast_2d(world_points)
        if world_points.shape[1] != 3:
            world_points = world_points.T

        # Compute camera pose
        R_heading = self.rotation_matrix_yaw(heading_deg)
        cam_origin_world = antenna_pos_world + R_heading @ self.gps_to_cam_offset_body

        # Plot common elements in top view
        ax_top.scatter(*antenna_pos_world[:2], color='blue', label='GPS Antenna')
        ax_top.scatter(*cam_origin_world[:2], color='green', label='Camera Origin')

        # Draw vehicle centered at antenna
        vehicle_length = 4.5
        vehicle_width = 2.0
        vehicle_corners_body = np.array([
            [vehicle_length / 2, vehicle_width / 2, 0],
            [vehicle_length / 2, -vehicle_width / 2, 0],
            [-vehicle_length / 2, -vehicle_width / 2, 0],
            [-vehicle_length / 2, vehicle_width / 2, 0],
            [vehicle_length / 2, vehicle_width / 2, 0]
        ]).T
        vehicle_world = (R_heading @ vehicle_corners_body).T + antenna_pos_world
        ax_top.plot(vehicle_world[:, 0], vehicle_world[:, 1], color='orange',
                    label='Vehicle')

        # Draw heading vector
        heading_vec = R_heading @ np.array([5.0, 0, 0])
        ax_top.quiver(*antenna_pos_world[:2], heading_vec[0], heading_vec[1],
                      color='cyan', scale=1, scale_units='xy', angles='xy',
                      label='Heading')

        # Draw camera direction vector
        rotation_from_camera_to_body = self.rotation_matrix_zyx(yaw, pitch, roll)
        logger.debug(f"Rotation from camera to body:\n{rotation_from_camera_to_body}")
        # start from a vector in camera frame, first rotate to frame same as body frame, then rotate to world frame
        cam_heading = R_heading @ rotation_from_camera_to_body @ self.R_body_to_cam.T @ np.array([0, 0, 5.0])
        logger.debug(f"Camera heading vector in world coords: {cam_heading}")
        ax_top.quiver(*cam_origin_world[:2], cam_heading[0], cam_heading[1],
                      color='red', scale=1, scale_units='xy', angles='xy',
                      label=f'Camera (y={yaw:.3f}°, p={pitch:.3f}°, r={roll:.3f}°)')

        # Draw ROI and points in both views
        # Process each world point
        colors = plt.cm.rainbow(np.linspace(0, 1, len(world_points)))
        for world_point, color in zip(world_points, colors):
            # Draw ROI along heading direction
            dx = roi_length / 2
            dy = roi_width / 2
            # Define ROI in vehicle frame first
            roi_corners_vehicle = np.array([
                [dx, dy, 0],
                [dx, -dy, 0],
                [-dx, -dy, 0],
                [-dx, dy, 0],
                [dx, dy, 0]
            ]).T
            # Rotate ROI to world frame based on heading
            roi_corners_local = R_heading @ roi_corners_vehicle
            roi_world = (roi_corners_local.T + world_point).T

            # --- TOP VIEW (2D XY) ---
            ax_top.plot(roi_world[0], roi_world[1], color=color, alpha=0.6)
            ax_top.scatter(*world_point[:2], color=color, label=f'Target Point {len(ax_top.collections)}')

            # --- IMAGE VIEW ---
            cam_coords = [
                self.world_to_camera(pt, antenna_pos_world, heading_deg,
                                     yaw, pitch, roll)
                for pt in roi_world.T
            ]
            img_pts = [self.project_point_to_image(pt) for pt in cam_coords if pt is not None]
            img_pts = np.array([pt for pt in img_pts if pt is not None])
            if len(img_pts) > 0:
                ax_img.plot(img_pts[:, 0], img_pts[:, 1], color=color,
                            alpha=0.6)
                ax_img.scatter(img_pts[:, 0], img_pts[:, 1], color=color,
                               label=f'Target {len(ax_img.collections)}')

        # Project world points to image view
        # Draw lane lines
        lane_width = 4.0  # meters
        # 1. Determine the farthest distance to the point(s) of interest
        if world_points.ndim == 1:
            max_dist = np.linalg.norm(world_points[:2] - antenna_pos_world[:2])
        else:
            max_dist = np.linalg.norm(world_points[:, :2] - antenna_pos_world[:2], axis=1).max()
        lane_max_dist = max(20.0, max_dist + 5.0)  # Add buffer for visualization

        # Lists to store left and right lane image points
        left_lane_img_pts = []
        right_lane_img_pts = []
        
        for offset in [-lane_width / 2, lane_width / 2]:
            lane_start = antenna_pos_world + R_heading @ np.array([0.0, offset, -antenna_pos_world[2]])
            # Get maximum distance from antenna to target point in the heading direction
            target_dist = np.linalg.norm(world_points[:, :2] - antenna_pos_world[:2], axis=1).max()
            dist = max(20.0, target_dist + 5.0)  # Add buffer for visualization
            lane_end = antenna_pos_world + R_heading @ np.array([lane_max_dist, offset, -antenna_pos_world[2]])
            ax_top.plot([lane_start[0], lane_end[0]], [lane_start[1], lane_end[1]], color='gray',
                        linestyle='--', label='Lane Boundary' if offset == -1.5 else None)

            distances = np.arange(0, lane_max_dist + 1e-3, 1.0)
            lane_img_pts = []
            for dist in distances:
                world_pt = antenna_pos_world + R_heading @ np.array([dist, offset, -antenna_pos_world[2]])
                cam_pt = self.world_to_camera(world_pt, antenna_pos_world, heading_deg, yaw, pitch, roll)
                img_pt = self.project_point_to_image(cam_pt)
                if img_pt is not None:
                    lane_img_pts.append(img_pt)
            lane_img_pts = np.array(lane_img_pts)
            if len(lane_img_pts) > 1:
                ax_img.plot(lane_img_pts[:, 0], lane_img_pts[:, 1],
                            color='gray', linestyle='--', linewidth=2, label='Lane Boundary' if offset == -lane_width / 2 else None)
            # Store points for each lane separately
            # note that the left and right lane projection may be behind the camera, when using it, need to filter out those beyond FOV
            if offset == -lane_width / 2:
                left_lane_img_pts = lane_img_pts
            else:
                right_lane_img_pts = lane_img_pts

            # Plot cross sections (perpendicular lines) along the lane centerline every ~5 meters
            cross_section_spacing = 5.0
            num_cross_sections = int(lane_max_dist // cross_section_spacing)
            for i in range(1, num_cross_sections + 1):
                dist = i * cross_section_spacing
                # Center point on the lane centerline
                center_world = antenna_pos_world + R_heading @ np.array([dist, 0.0, -antenna_pos_world[2]])
                # Define cross section endpoints in world coordinates (perpendicular to heading)
                left_world = center_world + R_heading @ np.array([0.0, -lane_width / 2, 0.0])
                right_world = center_world + R_heading @ np.array([0.0, lane_width / 2, 0.0])
                # Plot in top view
                ax_top.plot([left_world[0], right_world[0]], [left_world[1], right_world[1]], color='magenta', linestyle=':', linewidth=1)
                # Project to image and plot in image view
                left_cam = self.world_to_camera(left_world, antenna_pos_world, heading_deg, yaw, pitch, roll)
                right_cam = self.world_to_camera(right_world, antenna_pos_world, heading_deg, yaw, pitch, roll)
                left_img = self.project_point_to_image(left_cam)
                right_img = self.project_point_to_image(right_cam)
                if left_img is not None and right_img is not None:
                    ax_img.plot([left_img[0], right_img[0]], [left_img[1], right_img[1]], color='magenta', linestyle=':', linewidth=1)
                    # Optionally, mark the cross section index on both views for correspondence
                    ax_top.text(center_world[0], center_world[1], f'{dist:.0f}m', color='magenta', fontsize=8, ha='center', va='bottom')
                    ax_img.text((left_img[0] + right_img[0]) / 2, (left_img[1] + right_img[1]) / 2, f'{dist:.0f}m', color='magenta', fontsize=8, ha='center', va='bottom')

        # Draw camera field of view polygon
        fov_polygon = self.get_camera_fov_polygon(antenna_pos_world, heading_deg,
                                                  yaw, pitch, roll,
                                                  max_distance=lane_max_dist,
                                                  ground_height=0.0)
        ax_top.plot(fov_polygon[:, 0], fov_polygon[:, 1], color='purple',
                    linestyle='-', linewidth=2, alpha=0.7, label='Camera FOV')
        ax_top.fill(fov_polygon[:, 0], fov_polygon[:, 1], color='purple', alpha=0.1)

        # verify crack size estimation algorithm using the pixel to world back - projection
        pixel_distance = 50
        # Sample points within the area surrounded by lane boundary image points
        # Sampling crack points based on lane boundaries
        if len(left_lane_img_pts) > 0 and len(right_lane_img_pts) > 0:
            # Get y bounds for valid sampling region and cap at image boundaries
            y_min = max(left_lane_img_pts[0, 1], right_lane_img_pts[0, 1])
            y_max = min(left_lane_img_pts[-1, 1], right_lane_img_pts[-1, 1])
            
            # Cap at image boundaries (cy is center, so *2 gives full height/width)
            y_min = max(0, min(y_min, self.intrinsics['cy'] * 2))
            y_max = max(0, min(y_max, self.intrinsics['cy'] * 2))

            # Sample first point about 1/3 up from bottom
            v1 = y_min + (y_max - y_min) * (2.0 / 3.0 * np.random.random() + 1.0 / 3.0)

            # Find closest y-values in both lanes
            left_idx = np.searchsorted(left_lane_img_pts[:, 1], v1)
            right_idx = np.searchsorted(right_lane_img_pts[:, 1], v1)

            # Get x-coordinates at those indices
            if left_idx >= len(left_lane_img_pts):
                left_idx = len(left_lane_img_pts) - 1
            if right_idx >= len(right_lane_img_pts):
                right_idx = len(right_lane_img_pts) - 1

            left_x = max(0, min(left_lane_img_pts[left_idx, 0], self.intrinsics['cx'] * 2))
            right_x = max(0, min(right_lane_img_pts[right_idx, 0], self.intrinsics['cx'] * 2))

            # Sample first point randomly between lane boundaries
            u1 = left_x + (right_x - left_x) * np.random.random()

            # Sample second point in a circle with radius pixel_distance
            angle = 2 * np.pi * np.random.random()
            u2 = u1 + pixel_distance * np.cos(angle)
            v2 = v1 + pixel_distance * np.sin(angle)

            # Ensure second point stays within lane boundaries and image
            v2 = max(0, min(v2, y_min))
            v2 = min(v2, self.intrinsics['cy'] * 2)

            # Find x bounds for second point
            left_idx2 = np.searchsorted(left_lane_img_pts[:, 1], v2)
            right_idx2 = np.searchsorted(right_lane_img_pts[:, 1], v2)

            if left_idx2 >= len(left_lane_img_pts):
                left_idx2 = len(left_lane_img_pts) - 1
            if right_idx2 >= len(right_lane_img_pts):
                right_idx2 = len(right_lane_img_pts) - 1

            left_x2 = max(0, min(left_lane_img_pts[left_idx2, 0], self.intrinsics['cx'] * 2))
            right_x2 = max(0, min(right_lane_img_pts[right_idx2, 0], self.intrinsics['cx'] * 2))

            u2 = left_x2 + (right_x2 - left_x2) * np.random.random()
            logger.info(f"Sampled crack points: ({u1:.2f}, {v1:.2f}) and ({u2:.2f}, {v2:.2f})")
        else:
            logger.info("No valid lane boundaries found for sampling crack points.")
            # Fallback values
            u1 = self.intrinsics['cx']
            v1 = self.intrinsics['cy'] + 200
            u2 = u1
            v2 = v1 - 50

        # Back - project to world coordinates
        world_pt1 = self.camera_pixel_to_world(
            [u1, v1],
            antenna_pos_world=antenna_pos_world,
            heading_deg=heading_deg,
            yaw=yaw,
            pitch=pitch,
            roll=roll
        )

        world_pt2 = self.camera_pixel_to_world(
            [u2, v2],
            antenna_pos_world=antenna_pos_world,
            heading_deg=heading_deg,
            yaw=yaw,
            pitch=pitch,
            roll=roll
        )

        if world_pt1 is not None and world_pt2 is not None:
            # Print distances
            dist = np.linalg.norm(world_pt2[:2] - world_pt1[:2])
            logger.info(f"Distance between back-projected points: {dist:.2f}m")

            # Calculate distances
            dist_world = np.linalg.norm(world_pt2[:2] - world_pt1[:2])
            dist_pixel = np.sqrt((u2 - u1)**2 + (v2 - v1)**2)

            # Plot in top view
            ax_top.plot([world_pt1[0], world_pt2[0]],
                        [world_pt1[1], world_pt2[1]],
                        'k-', linewidth=3, label=f'Sample crack projected back ({dist_world:.2f}m)')

            # Plot in image view
            ax_img.plot([u1, u2], [v1, v2],
                        'k-', linewidth=3, label=f'Sample crack in image ({dist_pixel:.2f}px)')

        # Finalize plots
        ax_top.set_xlabel('East')
        ax_top.set_ylabel('North')
        ax_top.set_title('Top View (World XY)')
        ax_top.set_aspect('equal')
        ax_top.legend(loc='upper right', ncol=2)

        ax_img.set_xlim(0, self.intrinsics['cx'] * 2)
        ax_img.set_ylim(self.intrinsics['cy'] * 2, 0)
        ax_img.set_xlabel('u (px)')
        ax_img.set_ylabel('v (px)')
        ax_img.set_title('Image View (Projection)')
        ax_img.legend()

        plt.tight_layout()
        plt.show()
        return fig, ax_top, ax_img


if __name__ == "__main__":
    # setup logging

    # Set up logging format and level
    parser = argparse.ArgumentParser(description='Camera model for vehicle mounted camera.')
    parser.add_argument('--log_level', type=str, default='INFO',
                        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
                        help='Set the logging level')
    args = parser.parse_args()

    logger.setLevel(getattr(logging, args.log_level))

    '''
    # Notes:
    # - heading_deg: vehicle heading angle (0 = North, 90 = East, 180 = South, etc.)
    # - yaw: rotation around camera Z axis (forward)
    # - pitch: rotation around camera X axis (right)
    # - roll: rotation around camera Y axis (down)
    # - camera frame convention: Z-forward, X-right, Y-down (when yaw=pitch=roll=0)
    '''
    # In vehicle body frame, forward, left and up
    gps_to_cam_offset_body = np.array([1.0, 0.1, -0.1])
    # Camera intrinsics from calibration
    # 2025 - 07-30 calibration on G - 319 + 12mm lens.
    # remark: intrinsics can be approximated using camera specs.
    # see utility/camera/Intrinsic_parameter.md
    intrinsics = {
        'fx': 3406.175114,
        'fy': 3423.223242,
        'cx': 1080.429855,
        'cy': 785.284572,
        'gamma': 0.0
    }
    model = CameraModel(gps_to_cam_offset_body, intrinsics)

    # TODO: generate a random world point from the camera field of view
    antenna_pos_world = np.array([0.0, 0.0, 1.3])  # GPS world coordinates in ENU
    world_point = np.array([20.0 + 0.05 * np.random.uniform(-1, 1), 0.0, 0.0])        # Target point on road

    # Heading in degrees (counter clockwise is positive, 0 = East, 90 = North, etc.)
    # heading_deg = 0.0   # Facing east
    heading_deg = np.random.uniform(-10, 10)  # Random heading for testing

    # Yaw, pitch, and roll angles in degrees in body frame
    yaw_in_body_frame = - heading_deg
    pitch_in_body_frame = 0.0 # positive is down
    roll_in_body_frame = 0.0 # positive is right

    R_camera = model.rotation_matrix_zyx(yaw_in_body_frame, pitch_in_body_frame, roll_in_body_frame)

    logger.debug(f"Camera rotation matrix:\n{R_camera}")

    cam_coords = model.world_to_camera(world_point, antenna_pos_world, heading_deg, yaw_in_body_frame, pitch_in_body_frame, roll_in_body_frame)

    # plot scenario
    # Find existing test folders and create new one with incremental number
    base_folder = 'camera_model_test'
    existing_folders = [d for d in os.listdir('samples/data') if d.startswith(base_folder)]
    next_num = 1
    if existing_folders:
        nums = [int(f.split('_')[-1]) for f in existing_folders if f.split('_')[-1].isdigit()]
        if nums:
            next_num = max(nums) + 1
    output_folder = os.path.join('samples', 'data', f'{base_folder}_{next_num:02d}')
    os.makedirs(output_folder, exist_ok=True)

    # ---- example of how to use the model as quick test of projection and back-projection -----------
    logger.info("\nTesting projection and back-projection:")
    # First project world point to image
    cam_coords = model.world_to_camera(world_point, antenna_pos_world, heading_deg,
                                       yaw_in_body_frame, pitch_in_body_frame, roll_in_body_frame)
    uv = model.project_point_to_image(cam_coords)
    logger.info(f"World point {world_point} projects to image coordinates {uv}")
    
    # Then project back to world
    world_point_back = model.camera_pixel_to_world(
        uv,
        antenna_pos_world=antenna_pos_world,
        heading_deg=heading_deg,
        yaw=yaw_in_body_frame,
        pitch=pitch_in_body_frame,
        roll=roll_in_body_frame
    )
    
    # Compare results
    if world_point_back is not None:
        # remark: Using assert statements in production code is not recommended as they can be disabled with the -O flag.
        # Consider using a proper exception like RuntimeError or converting this to a warning/error log message for production robustness.
        # assert np.allclose(world_point[:2], world_point_back[:2], atol=1e-8), "Back-projected point does not match original point"
        if not np.allclose(world_point[:2], world_point_back[:2], atol=1e-8):
            logger.error("Back-projected point does not match original point")
            raise RuntimeError("Back-projected point does not match original point")
        error = np.linalg.norm(world_point - world_point_back)
        logger.info(f"Back-projected world point: {world_point_back}")
        logger.info(f"Projection error: {error:.8f} meters")

    # -------------------------------------------------------------------------------------------------
    
    # plot top view and image view comparison
    fig, ax_top, ax_img = model.plot_comparison_view(world_point, antenna_pos_world, heading_deg, yaw_in_body_frame, pitch_in_body_frame, roll_in_body_frame)
    fig.savefig(os.path.join(output_folder, 'top_view_vs_image_view.png'))

    # show coordinate systems comparison in 3D
    fig_cord, ax_cord = model.plot_coordinate_systems(antenna_pos_world, heading_deg, yaw_in_body_frame, pitch_in_body_frame, roll_in_body_frame)
    fig_cord.savefig(os.path.join(output_folder, "coordinate_comparison.png"))
