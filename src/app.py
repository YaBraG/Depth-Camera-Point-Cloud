from __future__ import annotations

import tkinter as tk
from tkinter import ttk

import numpy as np

from . import config
from .file_io import save_open3d_point_cloud
from .map_builder import MapBuilder
from .pointcloud_math import create_point_cloud_from_rgbd, numpy_to_open3d_point_cloud
from .realsense_camera import RealSenseCamera
from .viewer_open3d import Open3DLiveViewer


class RealSenseMapperApp:
    MODE_IDLE = "Idle"
    MODE_RGB = "RGB"
    MODE_DEPTH = "Depth"
    MODE_ALIGNED = "Aligned RGB-D"
    MODE_POINT_CLOUD = "Live Point Cloud"

    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("RealSense D435 3D Mapper")
        self.root.geometry("520x560")

        self.camera = RealSenseCamera()
        self.map_builder = MapBuilder()
        self.viewer = Open3DLiveViewer()

        self.mode = self.MODE_IDLE
        self.streaming = False
        self.auto_mapping = False
        self.frame_counter = 0

        self.current_point_cloud = None
        self.current_point_count = 0
        self.last_saved_path = ""
        self.last_error = ""

        self._cv2 = None
        self._build_ui()
        self._bind_shortcuts()
        self.root.protocol("WM_DELETE_WINDOW", self.exit_app)
        self._schedule_update()

    def run(self) -> None:
        self.root.mainloop()

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill=tk.BOTH, expand=True)

        title = ttk.Label(outer, text="RealSense D435 3D Mapper", font=("Segoe UI", 16, "bold"))
        title.pack(anchor=tk.W, pady=(0, 10))

        buttons = ttk.Frame(outer)
        buttons.pack(fill=tk.X)

        button_defs = [
            ("Test Camera", self.test_camera),
            ("Show RGB", lambda: self.start_mode(self.MODE_RGB)),
            ("Show Depth", lambda: self.start_mode(self.MODE_DEPTH)),
            ("Show Aligned RGB-D", lambda: self.start_mode(self.MODE_ALIGNED)),
            ("Show Live Point Cloud", lambda: self.start_mode(self.MODE_POINT_CLOUD)),
            ("Add Frame to Map", self.add_frame_to_map),
            ("Toggle Auto Mapping", self.toggle_auto_mapping),
            ("Save Current Point Cloud", self.save_current_point_cloud),
            ("Save Full Map", self.save_full_map),
            ("Clear Map", self.clear_map),
            ("ICP Alignment Test", self.run_icp_alignment_test),
            ("Stop Stream", self.stop_stream),
            ("Exit", self.exit_app),
        ]

        for index, (text, command) in enumerate(button_defs):
            row = index // 2
            column = index % 2
            button = ttk.Button(buttons, text=text, command=command)
            button.grid(row=row, column=column, sticky="ew", padx=4, pady=4)

        buttons.columnconfigure(0, weight=1)
        buttons.columnconfigure(1, weight=1)

        status_frame = ttk.LabelFrame(outer, text="Status", padding=10)
        status_frame.pack(fill=tk.BOTH, expand=True, pady=(12, 0))

        self.status_vars = {
            "mode": tk.StringVar(value="Current mode: Idle"),
            "streaming": tk.StringVar(value="Streaming: no"),
            "auto": tk.StringVar(value="Auto mapping: off"),
            "current_points": tk.StringVar(value="Current point cloud: 0 points"),
            "map_points": tk.StringVar(value="Accumulated map: 0 points"),
            "saved": tk.StringVar(value="Last saved file: none"),
            "message": tk.StringVar(value="Ready"),
        }

        for var in self.status_vars.values():
            ttk.Label(status_frame, textvariable=var, wraplength=460).pack(anchor=tk.W, pady=2)

        help_text = "Shortcuts: q/ESC stop, s save cloud, m add map frame, a auto map, c clear, p save map, h help"
        ttk.Label(outer, text=help_text, wraplength=480, foreground="#555555").pack(anchor=tk.W, pady=(10, 0))

    def _bind_shortcuts(self) -> None:
        self.root.bind("<Escape>", lambda event: self.stop_stream())
        self.root.bind("q", lambda event: self.stop_stream())
        self.root.bind("s", lambda event: self.save_current_point_cloud())
        self.root.bind("m", lambda event: self.add_frame_to_map())
        self.root.bind("a", lambda event: self.toggle_auto_mapping())
        self.root.bind("c", lambda event: self.clear_map())
        self.root.bind("p", lambda event: self.save_full_map())
        self.root.bind("h", lambda event: self.print_help())

    def _import_cv2(self):
        if self._cv2 is not None:
            return self._cv2
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("OpenCV is not installed. Run: py -3.12 -m pip install opencv-python") from exc
        self._cv2 = cv2
        return cv2

    def test_camera(self) -> None:
        try:
            message = self.camera.test_connection()
            self._set_message(message)
        except Exception as exc:
            self._set_error(exc)

    def start_mode(self, mode: str) -> None:
        try:
            self.stop_stream(close_camera=True)
            self.camera.start()
            self.mode = mode
            self.streaming = True
            self.frame_counter = 0
            self._set_message("Streaming started")
        except Exception as exc:
            self._set_error(exc)
            self.stop_stream()

    def stop_stream(self, close_camera: bool = True) -> None:
        self.streaming = False
        self.mode = self.MODE_IDLE
        self.viewer.close()
        try:
            cv2 = self._import_cv2()
            cv2.destroyAllWindows()
        except Exception:
            pass
        if close_camera:
            try:
                self.camera.stop()
            except Exception as exc:
                self._set_error(exc)
                return
        self._set_message("Streaming stopped")

    def _schedule_update(self) -> None:
        self._update_stream_once()
        self._refresh_status()
        self.root.after(15, self._schedule_update)

    def _update_stream_once(self) -> None:
        if not self.streaming:
            return
        try:
            frames = self.camera.get_aligned_frames()
            if frames is None:
                return

            self.frame_counter += 1
            if self.mode in (self.MODE_RGB, self.MODE_DEPTH, self.MODE_ALIGNED):
                self._show_cv_preview(frames)

            if self.mode == self.MODE_POINT_CLOUD or self.auto_mapping:
                self._update_current_point_cloud(frames)

            if self.mode == self.MODE_POINT_CLOUD:
                is_open = self.viewer.update(self.current_point_cloud)
                if not is_open:
                    self.stop_stream()
                    self._set_message("Open3D viewer closed; app is still running")

            if self.auto_mapping and self.frame_counter % config.AUTO_ACCUMULATE_EVERY_N_FRAMES == 0:
                self.add_frame_to_map(show_errors=False)

            self._handle_cv_keyboard()
        except Exception as exc:
            self._set_error(exc)
            self.stop_stream()

    def _show_cv_preview(self, frames) -> None:
        cv2 = self._import_cv2()
        color_image_bgr = frames["color_image_bgr"]
        depth_image_m = frames["depth_image_m"]

        if self.mode == self.MODE_RGB:
            cv2.imshow("RGB Stream - press q or ESC to stop", color_image_bgr)
            return

        depth_preview = self._make_depth_preview(depth_image_m)
        if self.mode == self.MODE_DEPTH:
            cv2.imshow("Depth Stream - press q or ESC to stop", depth_preview)
            return

        aligned_preview = np.hstack((color_image_bgr, depth_preview))
        cv2.imshow("Aligned RGB-D - press q or ESC to stop", aligned_preview)

    def _make_depth_preview(self, depth_image_m: np.ndarray) -> np.ndarray:
        cv2 = self._import_cv2()
        clipped_depth = np.clip(depth_image_m, config.MIN_DEPTH_M, config.MAX_DEPTH_M)
        normalized_depth = (clipped_depth - config.MIN_DEPTH_M) / (config.MAX_DEPTH_M - config.MIN_DEPTH_M)
        depth_8bit = (255.0 * (1.0 - normalized_depth)).astype(np.uint8)
        return cv2.applyColorMap(depth_8bit, cv2.COLORMAP_TURBO)

    def _update_current_point_cloud(self, frames) -> None:
        points_camera_m, colors_rgb = create_point_cloud_from_rgbd(
            frames["depth_image_m"],
            frames["color_image_bgr"],
            frames["intrinsics"],
        )
        self.current_point_cloud = numpy_to_open3d_point_cloud(points_camera_m, colors_rgb)
        if len(self.current_point_cloud.points) > 0:
            self.current_point_cloud = self.current_point_cloud.voxel_down_sample(config.VOXEL_SIZE_M)
        self.current_point_count = len(self.current_point_cloud.points)

    def _handle_cv_keyboard(self) -> None:
        if self._cv2 is None:
            return
        key = self._cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            self.stop_stream()
        elif key == ord("s"):
            self.save_current_point_cloud()
        elif key == ord("m"):
            self.add_frame_to_map()
        elif key == ord("a"):
            self.toggle_auto_mapping()
        elif key == ord("c"):
            self.clear_map()
        elif key == ord("p"):
            self.save_full_map()
        elif key == ord("h"):
            self.print_help()

    def add_frame_to_map(self, show_errors: bool = True) -> None:
        try:
            if self.current_point_cloud is None or len(self.current_point_cloud.points) == 0:
                frames = self.camera.get_aligned_frames() if self.camera.is_streaming else None
                if frames is not None:
                    self._update_current_point_cloud(frames)
            point_count = self.map_builder.add_frame(self.current_point_cloud)
            self._set_message(f"Added frame to map. Map now has {point_count} points")
        except Exception as exc:
            if show_errors:
                self._set_error(exc)

    def toggle_auto_mapping(self) -> None:
        self.auto_mapping = not self.auto_mapping
        state = "on" if self.auto_mapping else "off"
        self._set_message(f"Auto mapping turned {state}")

    def save_current_point_cloud(self) -> None:
        try:
            path = save_open3d_point_cloud(self.current_point_cloud, "current_cloud")
            self.last_saved_path = str(path)
            self._set_message("Point cloud saved")
        except Exception as exc:
            self._set_error(exc)

    def save_full_map(self) -> None:
        try:
            path = save_open3d_point_cloud(self.map_builder.get_map(), "accumulated_map")
            self.last_saved_path = str(path)
            self._set_message("Accumulated map saved")
        except Exception as exc:
            self._set_error(exc)

    def clear_map(self) -> None:
        try:
            self.map_builder.clear()
            self._set_message("Map cleared")
        except Exception as exc:
            self._set_error(exc)

    def run_icp_alignment_test(self) -> None:
        if not config.USE_ICP:
            self._set_message("ICP is off by default. Set USE_ICP = True in src/config.py to test alignment.")
            return
        try:
            if self.map_builder.previous_cloud is None:
                self.add_frame_to_map()
                self._set_message("ICP baseline frame added. Move the camera slightly, then press ICP Alignment Test again.")
                return
            self.add_frame_to_map()
            self._set_message("ICP alignment test added the current frame to the map")
        except Exception as exc:
            self._set_error(exc)

    def print_help(self) -> None:
        help_lines = [
            "RealSense D435 3D Mapper shortcuts:",
            "q or ESC: stop current stream / close preview",
            "s: save current point cloud",
            "m: add current frame to accumulated map",
            "a: toggle automatic mapping",
            "c: clear map",
            "p: save accumulated map",
            "h: print help",
        ]
        print("\n".join(help_lines))
        self._set_message("Help printed to terminal")

    def exit_app(self) -> None:
        self.streaming = False
        self.viewer.close()
        try:
            if self._cv2 is not None:
                self._cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            self.camera.stop()
        finally:
            self.root.destroy()

    def _set_message(self, message: str) -> None:
        self.last_error = ""
        self.status_vars["message"].set(message)
        self._refresh_status()

    def _set_error(self, exc: Exception) -> None:
        self.last_error = f"Error: {exc}"
        self.status_vars["message"].set(self.last_error)
        self._refresh_status()

    def _refresh_status(self) -> None:
        self.status_vars["mode"].set(f"Current mode: {self.mode}")
        self.status_vars["streaming"].set(f"Streaming: {'yes' if self.streaming else 'no'}")
        self.status_vars["auto"].set(f"Auto mapping: {'on' if self.auto_mapping else 'off'}")
        self.status_vars["current_points"].set(f"Current point cloud: {self.current_point_count} points")
        self.status_vars["map_points"].set(f"Accumulated map: {self.map_builder.point_count()} points")
        self.status_vars["saved"].set(f"Last saved file: {self.last_saved_path or 'none'}")
