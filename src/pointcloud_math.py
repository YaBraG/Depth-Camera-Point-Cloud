from __future__ import annotations

import numpy as np

from . import config


def create_point_cloud_from_rgbd(depth_image_m: np.ndarray, color_image_bgr: np.ndarray | None, intrinsics) -> tuple[np.ndarray, np.ndarray | None]:
    """Convert aligned RGB-D images into 3D camera-frame points.

    This function is intentionally independent from RealSense pipeline code so it
    can later be reused by a ROS 2 node that receives depth images and intrinsics.
    """
    if depth_image_m is None:
        return np.empty((0, 3), dtype=np.float64), None

    fx = float(intrinsics.fx)
    fy = float(intrinsics.fy)
    cx = float(intrinsics.ppx)
    cy = float(intrinsics.ppy)

    image_height, image_width = depth_image_m.shape[:2]
    stride = max(1, int(config.POINT_STRIDE))

    v_pixels, u_pixels = np.mgrid[0:image_height:stride, 0:image_width:stride]
    z_depth_m = depth_image_m[0:image_height:stride, 0:image_width:stride]

    valid_depth_mask = np.isfinite(z_depth_m)
    valid_depth_mask &= z_depth_m >= config.MIN_DEPTH_M
    valid_depth_mask &= z_depth_m <= config.MAX_DEPTH_M

    u_pixel = u_pixels[valid_depth_mask].astype(np.float64)
    v_pixel = v_pixels[valid_depth_mask].astype(np.float64)
    z_depth_m = z_depth_m[valid_depth_mask].astype(np.float64)

    if z_depth_m.size == 0:
        return np.empty((0, 3), dtype=np.float64), None

    # ==============================
    # This is the RGB-D camera projection math.
    # It converts a 2D depth pixel into a 3D point in the camera coordinate frame.
    #
    # u_pixel, v_pixel = pixel location in the image
    # z_depth_m = measured depth in meters
    # fx, fy = focal lengths from the RealSense camera intrinsics
    # cx, cy = optical center / principal point
    #
    # x_camera_m = horizontal 3D position
    # y_camera_m = vertical 3D position
    # z_camera_m = forward depth
    # ==============================
    x_camera_m = (u_pixel - cx) * z_depth_m / fx
    y_camera_m = (v_pixel - cy) * z_depth_m / fy
    z_camera_m = z_depth_m

    points_camera_m = np.column_stack((x_camera_m, y_camera_m, z_camera_m)).astype(np.float64)

    colors_rgb = None
    if color_image_bgr is not None:
        sampled_color_bgr = color_image_bgr[0:image_height:stride, 0:image_width:stride]
        colors_bgr = sampled_color_bgr[valid_depth_mask].astype(np.float64) / 255.0
        colors_rgb = colors_bgr[:, ::-1]

    return points_camera_m, colors_rgb


def numpy_to_open3d_point_cloud(points_camera_m: np.ndarray, colors_rgb: np.ndarray | None = None):
    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError("Open3D is not installed. Run: py -3.12 -m pip install open3d") from exc

    point_cloud = o3d.geometry.PointCloud()
    if points_camera_m is None or len(points_camera_m) == 0:
        return point_cloud

    point_cloud.points = o3d.utility.Vector3dVector(points_camera_m)
    if colors_rgb is not None and len(colors_rgb) == len(points_camera_m):
        point_cloud.colors = o3d.utility.Vector3dVector(colors_rgb)
    return point_cloud


def transform_points(points_camera_m: np.ndarray, T_world_camera: np.ndarray) -> np.ndarray:
    """Move camera-frame points into a world/map frame with a 4x4 transform."""
    if points_camera_m is None or len(points_camera_m) == 0:
        return np.empty((0, 3), dtype=np.float64)

    # ==============================
    # This is the rigid body transform math.
    # A 3D point from the current camera frame is moved into the map/world frame:
    #
    # p_world = R_world_camera @ p_camera + t_world_camera
    #
    # In homogeneous coordinates:
    #
    # p_world_h = T_world_camera @ p_camera_h
    # ==============================
    points_camera_h = np.column_stack((points_camera_m, np.ones(len(points_camera_m))))
    points_world_h = (T_world_camera @ points_camera_h.T).T
    return points_world_h[:, :3]
