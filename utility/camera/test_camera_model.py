'''
# Unit tests for CameraModel class in camera_model.py
Written with the help of Copilot. (Verified)
run with
python3 -m pytest test_camera_model.py
'''
import logging
import numpy as np
import pytest

from camera_model import CameraModel

# Configure logger for tests
logger = logging.getLogger("test_camera_model")
logger.setLevel(logging.INFO)
if not logger.handlers:  # Avoid duplicate handlers
    handler = logging.StreamHandler()
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    handler.setFormatter(formatter)
    logger.addHandler(handler)

# Also configure the camera_model logger for test visibility
camera_logger = logging.getLogger("camera_model")
camera_logger.setLevel(logging.WARNING)  # Reduce noise during tests

TOL = 6  # decimal places for floating point comparisons


@pytest.fixture
def default_model():
    gps_to_cam_offset_body = np.array([1.0, 0.0, -1.5])
    intrinsics = {
        'fx': 800.0,
        'fy': 800.0,
        'cx': 640.0,
        'cy': 360.0,
        'gamma': 0.0
    }
    return CameraModel(gps_to_cam_offset_body, intrinsics)


def test_rotation_matrix_yaw_identity(default_model):
    logger.debug("Testing rotation matrix yaw identity (0 degrees)")
    R = default_model.rotation_matrix_yaw(0)
    expected = np.eye(3)
    logger.debug(f"Result matrix:\n{R}")
    logger.debug(f"Expected matrix:\n{expected}")
    np.testing.assert_almost_equal(R, expected, decimal=TOL)
    logger.debug("Yaw identity test passed")


def test_rotation_matrix_yaw_90(default_model):
    logger.debug("Testing rotation matrix yaw 90 degrees")
    R = default_model.rotation_matrix_yaw(90)
    expected = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    logger.debug(f"Result matrix:\n{R}")
    logger.debug(f"Expected matrix:\n{expected}")
    np.testing.assert_almost_equal(R, expected, decimal=TOL)
    logger.debug("Yaw 90° test passed")


def test_rotation_matrix_zyx_zero(default_model):
    R = default_model.rotation_matrix_zyx(0, 0, 0)
    expected = np.eye(3)
    np.testing.assert_almost_equal(R, expected, decimal=TOL)


def test_world_to_camera_facing_east(default_model):
    """Test world to camera transformation when vehicle is facing east"""
    logger.info("Testing world to camera transformation facing east")

    # east north up (ENU) coordinate system
    world_point = np.array([1.0, 10.0, 0.0])
    antenna = np.array([0.0, 0.0, 0.0])
    heading = 0.0  # facing east
    yaw = pitch = roll = 0.0

    logger.debug(f"World point: {world_point}")
    logger.debug(f"Antenna: {antenna}")
    logger.debug(f"Heading: {heading}° (facing east)")

    cam_coords = default_model.world_to_camera(
        world_point, antenna, heading, yaw, pitch, roll
    )
    expected = np.array([-10, -1.5, 0])

    logger.debug(f"Camera coordinates: {cam_coords}")
    logger.debug(f"Expected coordinates: {expected}")

    np.testing.assert_almost_equal(cam_coords, expected, decimal=TOL)
    logger.info("East-facing transformation test passed")


def test_world_to_camera_facing_north(default_model):
    """Test world to camera transformation when vehicle is facing north"""
    logger.info("Testing world to camera transformation facing north")

    # east north up (ENU) coordinate system
    world_point = np.array([1.0, 20.0, 0.0])
    antenna = np.array([0.0, 0.0, 0.0])
    heading = 90.0  # facing north
    yaw = pitch = roll = 0.0

    logger.debug(f"World point: {world_point}")
    logger.debug(f"Antenna: {antenna}")
    logger.debug(f"Heading: {heading}° (facing north)")

    cam_coords = default_model.world_to_camera(
        world_point, antenna, heading, yaw, pitch, roll
    )
    expected = np.array([1, -1.5, 19])

    logger.debug(f"Camera coordinates: {cam_coords}")
    logger.debug(f"Expected coordinates: {expected}")

    np.testing.assert_almost_equal(cam_coords, expected, decimal=TOL)
    logger.info("North-facing transformation test passed")


def test_project_point_to_image_center(default_model):
    """Test projection of point at camera center"""
    logger.info("Testing projection of center point to image")

    point_cam = np.array([0.0, 0.0, 2.0])
    logger.debug(f"Camera point: {point_cam}")

    uv = default_model.project_point_to_image(point_cam)
    expected = [640.0, 360.0]

    logger.debug(f"Projected pixel: {uv}")
    logger.debug(f"Expected pixel: {expected}")

    assert np.allclose(uv, expected)
    logger.info("Center projection test passed")


def test_project_point_to_image_off_center(default_model):
    """Test projection of off-center point"""
    logger.info("Testing projection of off-center point to image")

    point_cam = np.array([1.0, 1.0, 2.0])
    logger.debug(f"Camera point: {point_cam}")

    uv = default_model.project_point_to_image(point_cam)
    expected = [1040.0, 760.0]

    logger.debug(f"Projected pixel: {uv}")
    logger.debug(f"Expected pixel: {expected}")

    assert np.allclose(uv, expected)
    logger.info("Off-center projection test passed")


def test_project_point_behind_camera(default_model):
    """Test projection of point behind camera (should return None)"""
    logger.info("Testing projection of point behind camera")

    point_cam = np.array([0.0, 0.0, -1.0])
    logger.debug(f"Camera point (behind): {point_cam}")

    result = default_model.project_point_to_image(point_cam)
    logger.debug(f"Projection result: {result}")

    assert result is None
    logger.info("Behind camera test passed (correctly returned None)")


def test_camera_pixel_to_world_consistency_with_projection(default_model):
    """Test that pixel_to_world is consistent with world_to_camera and
    project_point_to_image"""
    logger.info("Testing camera pixel to world consistency with projection")

    # Start with a known world point
    original_world_point = np.array([5.0, 2.0, 0.0])
    antenna = np.array([0.0, 0.0, 1.5])
    heading = np.random.uniform(0, 10)  # random heading
    yaw, pitch, roll = np.random.uniform(-2, 2, size=3)  # small rotations

    logger.debug(f"Original world point: {original_world_point}")
    logger.debug(f"Antenna position: {antenna}")
    logger.debug(f"Vehicle heading: {heading:.2f}°")
    logger.debug(f"Camera orientation - yaw: {yaw:.2f}°, "
                 f"pitch: {pitch:.2f}°, roll: {roll:.2f}°")

    # Project to camera coordinates and then to image
    cam_coords = default_model.world_to_camera(
        original_world_point, antenna, heading, yaw, pitch, roll
    )
    logger.debug(f"Camera coordinates: {cam_coords}")

    if cam_coords[2] > 0:  # Only test if point is in front of camera
        pixel_coords = default_model.project_point_to_image(cam_coords)
        logger.debug(f"Pixel coordinates: {pixel_coords}")

        if pixel_coords is not None:
            # Now convert back from pixel to world
            reconstructed_world = default_model.camera_pixel_to_world(
                pixel_coords, 1.5, antenna, heading, yaw, pitch, roll)

            logger.debug(f"Reconstructed world point: {reconstructed_world}")

            # Should get back the original world point (within tolerance)
            if reconstructed_world is not None:
                error = np.linalg.norm(
                    reconstructed_world - original_world_point
                )
                logger.info(f"Reconstruction error: {error:.6f} meters")
                np.testing.assert_allclose(
                    reconstructed_world, original_world_point, atol=1e-3
                )
                logger.info("Consistency test passed!")
            else:
                logger.warning("Reconstructed world point is None")
        else:
            logger.warning("Pixel coordinates are None")
    else:
        logger.warning(f"Point is behind camera (z={cam_coords[2]:.3f})")
