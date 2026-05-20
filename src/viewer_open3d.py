from __future__ import annotations

from . import config


class Open3DLiveViewer:
    def __init__(self, window_name: str = "Live RealSense Point Cloud") -> None:
        self.window_name = window_name
        self.o3d = None
        self.visualizer = None
        self.display_cloud = None
        self.coordinate_frame = None
        self.geometry_added = False
        self.coordinate_frame_added = False
        self.is_open = False

    def _import_open3d(self):
        if self.o3d is not None:
            return self.o3d
        try:
            import open3d as o3d
        except ImportError as exc:
            raise RuntimeError("Open3D is not installed. Run: py -3.12 -m pip install open3d") from exc
        self.o3d = o3d
        return o3d

    def open(self) -> None:
        if self.is_open:
            return
        o3d = self._import_open3d()
        self.visualizer = o3d.visualization.Visualizer()
        self.visualizer.create_window(window_name=self.window_name, width=960, height=720)
        self.display_cloud = o3d.geometry.PointCloud()
        self.coordinate_frame = None
        self.coordinate_frame_added = False
        if config.SHOW_COORDINATE_FRAME:
            # Open3D axis colors: red = x axis, green = y axis, blue = z axis.
            self.coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=config.COORDINATE_FRAME_SIZE_M
            )
        self.geometry_added = False
        self.is_open = True

    def update(self, point_cloud) -> bool:
        if point_cloud is None:
            return self.is_open
        if not self.is_open:
            self.open()
        if self.visualizer is None:
            return False

        try:
            self.display_cloud.points = point_cloud.points
            self.display_cloud.colors = point_cloud.colors
            if not self.geometry_added:
                self.visualizer.add_geometry(self.display_cloud)
                if self.coordinate_frame is not None and not self.coordinate_frame_added:
                    self.visualizer.add_geometry(self.coordinate_frame, reset_bounding_box=False)
                    self.coordinate_frame_added = True
                self.geometry_added = True
            else:
                self.visualizer.update_geometry(self.display_cloud)

            still_open = self.visualizer.poll_events()
            self.visualizer.update_renderer()
            if not still_open:
                self.close()
            return still_open
        except RuntimeError:
            self.close()
            return False

    def close(self) -> None:
        if self.visualizer is not None:
            try:
                self.visualizer.destroy_window()
            except RuntimeError:
                pass
        self.visualizer = None
        self.display_cloud = None
        self.coordinate_frame = None
        self.geometry_added = False
        self.coordinate_frame_added = False
        self.is_open = False
