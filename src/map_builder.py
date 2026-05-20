from __future__ import annotations

import numpy as np

from . import config


class MapBuilder:
    def __init__(self) -> None:
        self.o3d = None
        self.accumulated_map = None
        self.previous_cloud = None
        self.T_world_camera = np.eye(4, dtype=np.float64)

    def _import_open3d(self):
        if self.o3d is not None:
            return self.o3d
        try:
            import open3d as o3d
        except ImportError as exc:
            raise RuntimeError("Open3D is not installed. Run: py -3.12 -m pip install open3d") from exc
        self.o3d = o3d
        if self.accumulated_map is None:
            self.accumulated_map = o3d.geometry.PointCloud()
        return o3d

    def clear(self) -> None:
        o3d = self._import_open3d()
        self.accumulated_map = o3d.geometry.PointCloud()
        self.previous_cloud = None
        self.T_world_camera = np.eye(4, dtype=np.float64)

    def point_count(self) -> int:
        if self.accumulated_map is None:
            return 0
        return len(self.accumulated_map.points)

    def add_frame(self, current_cloud) -> int:
        o3d = self._import_open3d()
        if current_cloud is None or len(current_cloud.points) == 0:
            raise RuntimeError("No current point cloud is available to add to the map.")

        cloud_to_add = o3d.geometry.PointCloud(current_cloud)

        if config.USE_ICP and self.previous_cloud is not None and len(self.previous_cloud.points) > 0:
            # ICP estimates a rigid transform between point clouds. This is only
            # a small alignment experiment, not full SLAM or odometry.
            icp_result = o3d.pipelines.registration.registration_icp(
                cloud_to_add,
                self.previous_cloud,
                config.ICP_MAX_CORRESPONDENCE_DISTANCE_M,
                np.eye(4, dtype=np.float64),
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            )
            T_world_camera = icp_result.transformation

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
            cloud_to_add.transform(T_world_camera)

        self.accumulated_map += cloud_to_add
        if len(self.accumulated_map.points) > 0:
            self.accumulated_map = self.accumulated_map.voxel_down_sample(config.VOXEL_SIZE_M)
            if len(self.accumulated_map.points) > 100:
                filtered_map, _ = self.accumulated_map.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
                self.accumulated_map = filtered_map

        self.previous_cloud = o3d.geometry.PointCloud(current_cloud)
        return self.point_count()

    def get_map(self):
        self._import_open3d()
        return self.accumulated_map
