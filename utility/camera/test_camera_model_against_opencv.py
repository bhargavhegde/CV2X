#!/usr/bin/env python3
"""
    Simple focused test to validate camera projection math step by step
    Created with the help of Copilot
"""

import os

import cv2
import matplotlib.pyplot as plt
import numpy as np

from camera_model import CameraModel


def simple_validation_test():
    """
    Simple test with known points to validate projection math
    """
    print("=== Simple Camera Projection Validation ===\n")

    # Camera model setup
    gps_to_cam_offset_body = np.array([1.0, 0.1, -0.1])
    intrinsics = {
        'fx': 3406.175114,
        'fy': 3423.223242,
        'cx': 1080.429855,
        'cy': 785.284572,
        'gamma': 0.0
    }
    model = CameraModel(gps_to_cam_offset_body, intrinsics)

    # Simple test case - point directly in front
    antenna_pos_world = np.array([0.0, 0.0, 1.5])  # Simplified origin
    world_point = np.array([10.0, 0.0, 0.0])       # 10m directly ahead on ground

    heading_deg = 0.0   # Facing north (X direction)
    yaw = 0.0
    pitch = 0.0
    roll = 0.0

    print("Test Setup:")
    print(f"  Antenna position: {antenna_pos_world}")
    print(f"  World point: {world_point}")
    print(f"  GPS to camera offset (body frame): {gps_to_cam_offset_body}")
    print(f"  Heading: {heading_deg}°, Yaw: {yaw}°, Pitch: {pitch}°, Roll: {roll}°")
    print()

    # Step 1: Manual calculation of camera coordinates
    print("=== Step-by-step Manual Calculation ===")

    # Camera position in world
    R_heading = model.rotation_matrix_yaw(heading_deg)
    print(f"Heading rotation matrix:\n{R_heading}")

    offset_world = R_heading @ gps_to_cam_offset_body
    camera_origin_world = antenna_pos_world + offset_world
    print(f"Camera origin in world: {camera_origin_world}")

    # Relative point
    rel_point = world_point - camera_origin_world
    print(f"Relative point (world frame): {rel_point}")

    # Apply rotations
    R_cam_pose = model.rotation_matrix_zyx(yaw, pitch, roll)
    print(f"Camera pose rotation matrix:\n{R_cam_pose}")

    R_total = model.R_body_to_cam @ R_cam_pose @ R_heading.T
    print(f"Total rotation matrix:\n{R_total}")

    camera_coords_manual = R_total @ rel_point
    print(f"Camera coordinates (manual): {camera_coords_manual}")

    # Step 2: Using the model's method
    print("\n=== Using CameraModel.world_to_camera() ===")
    camera_coords_model = model.world_to_camera(world_point, antenna_pos_world, heading_deg, yaw, pitch, roll)
    print(f"Camera coordinates (model): {camera_coords_model}")

    # Check consistency
    coord_diff = np.linalg.norm(camera_coords_manual - camera_coords_model)
    print(f"Difference between manual and model: {coord_diff:.10f}")

    # Step 3: Project to image using custom method
    print("\n=== Custom Image Projection ===")
    image_point_custom = model.project_point_to_image(camera_coords_model)
    print(f"Image point (custom): {image_point_custom}")

    # Step 4: Project using OpenCV
    print("\n=== OpenCV Projection ===")

    camera_matrix = np.array([
        [intrinsics['fx'], intrinsics.get('gamma', 0.0), intrinsics['cx']],
        [0, intrinsics['fy'], intrinsics['cy']],
        [0, 0, 1]
    ])
    print(f"Camera matrix:\n{camera_matrix}")

    dist_coeffs = np.zeros(4)
    rvec = np.zeros(3)  # No rotation (point already in camera frame)
    tvec = np.zeros(3)  # No translation

    # OpenCV expects (N, 1, 3) shape
    camera_point_opencv = camera_coords_model.reshape(1, 1, 3)

    image_points_opencv, _ = cv2.projectPoints(
        camera_point_opencv,
        rvec,
        tvec,
        camera_matrix,
        dist_coeffs
    )
    image_point_opencv = image_points_opencv[0, 0]
    print(f"Image point (OpenCV): {image_point_opencv}")

    # Step 5: Manual projection calculation
    print("\n=== Manual Projection Calculation ===")
    x, y, z = camera_coords_model
    print(f"Camera coordinates: X={x:.6f}, Y={y:.6f}, Z={z:.6f}")

    if z > 0:
        u_manual = intrinsics['fx'] * (x / z) + intrinsics['cx']
        v_manual = intrinsics['fy'] * (y / z) + intrinsics['cy']
        print(f"Manual projection: u={u_manual:.6f}, v={v_manual:.6f}")

        # Compare with camera matrix multiplication
        p_normalized = np.array([x / z, y / z, 1.0])
        p_image_homog = camera_matrix @ p_normalized
        print(f"Matrix multiplication result: {p_image_homog[:2]}")
    else:
        print("Point is behind camera (Z <= 0)")

    # Step 6: Compare all methods
    print("\n=== Final Comparison ===")
    if image_point_custom is not None and z > 0:
        print(f"Custom projection:  [{image_point_custom[0]:10.6f}, {image_point_custom[1]:10.6f}]")
        print(f"OpenCV projection:  [{image_point_opencv[0]:10.6f}, {image_point_opencv[1]:10.6f}]")
        print(f"Manual calculation: [{u_manual:10.6f}, {v_manual:10.6f}]")

        diff_custom_opencv = np.linalg.norm(image_point_custom - image_point_opencv)
        diff_manual_opencv = np.linalg.norm([u_manual, v_manual] - image_point_opencv)

        print("\nDifferences:")
        print(f"  Custom vs OpenCV:  {diff_custom_opencv:.8f} pixels")
        print(f"  Manual vs OpenCV:  {diff_manual_opencv:.8f} pixels")

        if diff_custom_opencv < 1e-6:
            print("✅ VALIDATION PASSED: Custom projection matches OpenCV")
        else:
            print("⚠️  VALIDATION FAILED: Significant difference detected")
    else:
        print("Cannot compare - point projection failed")


def test_multiple_scenarios():
    """Test different camera orientations and positions with randomized points"""
    print("\n" + "=" * 60)
    print("=== Testing Multiple Scenarios with Randomized Points ===")

    # Set random seed for reproducible results
    # np.random.seed(42)

    gps_to_cam_offset_body = np.array([1.0, 0.1, -0.1])
    intrinsics = {
        'fx': 3406.175114,
        'fy': 3423.223242,
        'cx': 1080.429855,
        'cy': 785.284572,
        'gamma': 0.0
    }
    model = CameraModel(gps_to_cam_offset_body, intrinsics)

    def generate_random_points_in_area(antenna_pos, heading_deg, area_type, num_points=5):
        """Generate random points in different areas relative to the antenna"""
        points = []
        R_heading = model.rotation_matrix_yaw(heading_deg)

        if area_type == 'forward':
            # Points in front of the vehicle (5-50m ahead, ±10m side)
            for _ in range(num_points):
                forward_dist = np.random.uniform(5, 50)
                side_offset = np.random.uniform(-10, 10)
                height = np.random.uniform(-0.5, 1.0)
                local_point = np.array([forward_dist, side_offset, height])
                world_point = antenna_pos + R_heading @ local_point
                points.append(world_point)

        elif area_type == 'near_field':
            # Close points (1-10m, all directions)
            for _ in range(num_points):
                forward_dist = np.random.uniform(1, 10)
                side_offset = np.random.uniform(-5, 5)
                height = np.random.uniform(-0.5, 1.0)
                local_point = np.array([forward_dist, side_offset, height])
                world_point = antenna_pos + R_heading @ local_point
                points.append(world_point)

        elif area_type == 'side_areas':
            # Points to the left and right sides
            for _ in range(num_points):
                forward_dist = np.random.uniform(5, 30)
                side_offset = np.random.choice([-1, 1]) * np.random.uniform(5, 20)
                height = np.random.uniform(-0.5, 1.0)
                local_point = np.array([forward_dist, side_offset, height])
                world_point = antenna_pos + R_heading @ local_point
                points.append(world_point)

        elif area_type == 'elevated':
            # Points at different elevations
            for _ in range(num_points):
                forward_dist = np.random.uniform(10, 40)
                side_offset = np.random.uniform(-8, 8)
                height = np.random.uniform(0.5, 3.0)  # Above camera level
                local_point = np.array([forward_dist, side_offset, height])
                world_point = antenna_pos + R_heading @ local_point
                points.append(world_point)

        elif area_type == 'random_global':
            # Completely random points in a larger area
            for _ in range(num_points):
                offset_x = np.random.uniform(-50, 100)
                offset_y = np.random.uniform(-50, 50)
                offset_z = np.random.uniform(-1, 2)
                world_point = antenna_pos + np.array([offset_x, offset_y, offset_z])
                points.append(world_point)

        return points

    test_scenarios = [
        {
            'name': 'Forward View - Random Points Ahead',
            'antenna_pos': np.array([0.0, 0.0, 1.5]),
            'heading': 0.0,
            'yaw': np.random.uniform(-2, 2),
            'pitch': np.random.uniform(-2, 2),
            'roll': np.random.uniform(-2, 2),
            'area_type': 'forward'
        },
        {
            'name': 'Pitched Camera - Near Field',
            'antenna_pos': np.array([0.0, 0.0, 1.5]),
            'heading': 0.0,
            'yaw': np.random.uniform(-2, 2),
            'pitch': np.random.uniform(-2, 2),
            'roll': np.random.uniform(-2, 2),
            'area_type': 'near_field'
        },
        {
            'name': 'Turned Vehicle - Side Areas',
            'antenna_pos': np.array([50.0, 25.0, 1.5]),
            'heading': 45.0,
            'yaw': np.random.uniform(-2, 2),
            'pitch': np.random.uniform(-2, 2),
            'roll': np.random.uniform(-2, 2),
            'area_type': 'side_areas'
        },
        {
            'name': 'Rolled Camera - Elevated Points',
            'antenna_pos': np.array([100.0, 50.0, 1.5]),
            'heading': 0.0,
            'yaw': np.random.uniform(-2, 2),
            'pitch': np.random.uniform(-2, 2),
            'roll': np.random.uniform(-2, 2),
            'area_type': 'elevated'
        },
        # {
        #     'name': 'Complex Orientation - Random Global',
        #     'antenna_pos': np.array([200.0, 100.0, 2.0]),
        #     'heading': 135.0,
        #     'yaw': np.random.uniform(-2, 2),
        #     'pitch': np.random.uniform(-2, 2),
        #     'roll': np.random.uniform(-2, 2),
        #     'area_type': 'random_global'
        # }
    ]

    camera_matrix = np.array([
        [intrinsics['fx'], intrinsics.get('gamma', 0.0), intrinsics['cx']],
        [0, intrinsics['fy'], intrinsics['cy']],
        [0, 0, 1]
    ])

    total_tests = 0
    total_passed = 0
    max_difference = 0.0
    all_differences = []

    for i, scenario in enumerate(test_scenarios):
        print(f"\n--- Scenario {i+1}: {scenario['name']} ---")

        # Generate random test points for this scenario
        test_points = generate_random_points_in_area(
            scenario['antenna_pos'],
            scenario['heading'],
            scenario['area_type'],
            num_points=8  # Test 8 random points per scenario
        )

        scenario_passed = 0
        scenario_total = 0

        for j, world_point in enumerate(test_points):
            # Get camera coordinates
            cam_coords = model.world_to_camera(
                world_point, scenario['antenna_pos'],
                scenario['heading'], scenario['yaw'], scenario['pitch'], scenario['roll']
            )

            if cam_coords[2] > 0:  # Point in front of camera
                scenario_total += 1
                total_tests += 1

                # Custom projection
                img_pt_custom = model.project_point_to_image(cam_coords)

                # OpenCV projection
                img_pts_opencv, _ = cv2.projectPoints(
                    cam_coords.reshape(1, 1, 3),
                    np.zeros(3), np.zeros(3),
                    camera_matrix, np.zeros(4)
                )
                img_pt_opencv = img_pts_opencv[0, 0]

                if img_pt_custom is not None:
                    diff = np.linalg.norm(img_pt_custom - img_pt_opencv)
                    all_differences.append(diff)
                    max_difference = max(max_difference, diff)

                    if j < 3:  # Print details for first 3 points only
                        print(f"  Point {j+1}: Custom=[{img_pt_custom[0]:7.2f}, {img_pt_custom[1]:7.2f}], "
                              f"OpenCV=[{img_pt_opencv[0]:7.2f}, {img_pt_opencv[1]:7.2f}], "
                              f"Diff={diff:.8f}")

                    if diff < 1e-6:
                        scenario_passed += 1
                        total_passed += 1

                else:
                    print(f"  Point {j+1}: Custom projection returned None")

        if scenario_total > 3:  # Show summary if we tested more than 3 points
            print(f"  ... (tested {scenario_total} total points)")

        print(f"  Scenario Result: {scenario_passed}/{scenario_total} passed")

        # Collect projection results for visualization
        projection_results = []
        for world_point in test_points:
            cam_coords = model.world_to_camera(
                world_point, scenario['antenna_pos'],
                scenario['heading'], scenario['yaw'], scenario['pitch'], scenario['roll']
            )

            if cam_coords[2] > 0:  # Point in front of camera
                # Custom projection
                img_pt_custom = model.project_point_to_image(cam_coords)

                # OpenCV projection
                img_pts_opencv, _ = cv2.projectPoints(
                    cam_coords.reshape(1, 1, 3),
                    np.zeros(3), np.zeros(3),
                    camera_matrix, np.zeros(4)
                )
                img_pt_opencv = img_pts_opencv[0, 0]

                projection_results.append({
                    'world_point': world_point,
                    'custom_projection': img_pt_custom,
                    'opencv_projection': img_pt_opencv
                })

        # Visualize this scenario using the camera model's built-in method
        visualize_test_scenario_simple(model, scenario, test_points, i + 1)

    # Overall summary
    print("=" * 60)
    print("=== OVERALL SUMMARY ===")
    print(f"Total tests: {total_tests}")
    print(f"Tests passed: {total_passed}")
    print(f"Success rate: {100.0 * total_passed / max(1, total_tests):.1f}%")
    print(f"Maximum difference: {max_difference:.8f} pixels")

    if all_differences:
        print(f"Mean difference: {np.mean(all_differences):.8f} pixels")
        print(f"Std difference: {np.std(all_differences):.8f} pixels")
        print(f"95th percentile: {np.percentile(all_differences, 95):.8f} pixels")

    if total_passed == total_tests and total_tests > 0:
        print("✅ ALL TESTS PASSED: Custom projection perfectly matches OpenCV")
    elif max_difference < 1e-3:
        print("✅ TESTS MOSTLY PASSED: Differences are negligible (<0.001 pixels)")
    else:
        print("⚠️  SOME TESTS FAILED: Significant differences detected")


def visualize_test_scenario_simple(model, scenario, test_points, scenario_index):
    """Simple visualization using the camera model's built-in plot_comparison_view method"""
    print(f"  Generating visualization for scenario {scenario_index}...")

    # Convert test points to numpy array for the plot_comparison_view method
    test_points = np.array(test_points)

    # Use the camera model's built-in visualization method
    try:
        fig, ax_top, ax_img = model.plot_comparison_view(
            world_points=test_points,
            antenna_pos_world=scenario['antenna_pos'],
            heading_deg=scenario['heading'],
            yaw=scenario['yaw'],
            pitch=scenario['pitch'],
            roll=scenario['roll'],
            roi_length=2.0,  # 2m ROI length around each point
            roi_width=1.5    # 1.5m ROI width
        )

        # Update the plot title to include scenario information
        ax_top.set_title(f'Scenario {scenario_index}: {scenario["name"]}\nTop View (World XY)')
        ax_img.set_title(f'Scenario {scenario_index}: {scenario["name"]}\nImage View (Projection)')

        # Create results directory if it doesn't exist
        target_dir = os.path.join('data', 'compare_with_cv2')
        os.makedirs(target_dir, exist_ok=True)

        # Save the figure
        fig.savefig(f'{target_dir}/scenario_{scenario_index}_{scenario["area_type"]}.png',
                    dpi=150, bbox_inches='tight')

        # Show the plot
        plt.show()

        print(f"  ✅ Visualization saved to {target_dir}/scenario_{scenario_index}_{scenario['area_type']}.png")

    except Exception as e:
        print(f"  ⚠️ Visualization failed: {e}")


if __name__ == "__main__":
    simple_validation_test()
    test_multiple_scenarios()
