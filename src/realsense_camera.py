from __future__ import annotations

import numpy as np

from . import config


class RealSenseCamera:
    def __init__(self) -> None:
        self.rs = None
        self.pipeline = None
        self.align = None
        self.profile = None
        self.depth_scale_m = 0.001
        self.is_streaming = False

    def _import_realsense(self):
        if self.rs is not None:
            return self.rs
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError("pyrealsense2 is not installed. Run: py -3.12 -m pip install pyrealsense2") from exc
        self.rs = rs
        return rs

    def test_connection(self) -> str:
        rs = self._import_realsense()
        context = rs.context()
        devices = context.query_devices()
        if len(devices) == 0:
            raise RuntimeError("RealSense camera not found. Check USB cable, power, and Intel RealSense Viewer.")

        device = devices[0]
        name = device.get_info(rs.camera_info.name)
        serial = device.get_info(rs.camera_info.serial_number)
        return f"Camera connected: {name} serial {serial}"

    def start(self) -> None:
        if self.is_streaming:
            return
        if config.DEMO_MODE:
            self.is_streaming = True
            return

        rs = self._import_realsense()
        self.test_connection()

        self.pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.depth, config.DEPTH_WIDTH, config.DEPTH_HEIGHT, rs.format.z16, config.FPS)
        rs_config.enable_stream(rs.stream.color, config.COLOR_WIDTH, config.COLOR_HEIGHT, rs.format.bgr8, config.FPS)

        self.profile = self.pipeline.start(rs_config)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale_m = float(depth_sensor.get_depth_scale())
        self.align = rs.align(rs.stream.color)
        self.is_streaming = True

    def stop(self) -> None:
        if config.DEMO_MODE:
            self.is_streaming = False
            return
        if self.pipeline is not None and self.is_streaming:
            self.pipeline.stop()
        self.pipeline = None
        self.align = None
        self.profile = None
        self.is_streaming = False

    def get_aligned_frames(self):
        if not self.is_streaming:
            self.start()

        if config.DEMO_MODE:
            return self._demo_frames()

        frames = self.pipeline.poll_for_frames()
        if not frames:
            return None

        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None

        depth_image_raw = np.asanyarray(depth_frame.get_data())
        color_image_bgr = np.asanyarray(color_frame.get_data())
        depth_image_m = depth_image_raw.astype(np.float32) * self.depth_scale_m
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        return {
            "depth_image_m": depth_image_m,
            "depth_image_raw": depth_image_raw,
            "color_image_bgr": color_image_bgr,
            "intrinsics": intrinsics,
        }

    def _demo_frames(self):
        height = config.DEPTH_HEIGHT
        width = config.DEPTH_WIDTH
        u_grid = np.linspace(-1.0, 1.0, width, dtype=np.float32)
        v_grid = np.linspace(-1.0, 1.0, height, dtype=np.float32)[:, None]
        depth_image_m = 1.2 + 0.2 * np.sin(4.0 * u_grid)[None, :] + 0.1 * np.cos(3.0 * v_grid)
        depth_image_m = depth_image_m.astype(np.float32)
        color_image_bgr = np.zeros((height, width, 3), dtype=np.uint8)
        color_image_bgr[:, :, 0] = np.clip((u_grid + 1.0) * 127.5, 0, 255).astype(np.uint8)
        color_image_bgr[:, :, 1] = np.clip((v_grid + 1.0) * 127.5, 0, 255).astype(np.uint8)
        color_image_bgr[:, :, 2] = 180

        class DemoIntrinsics:
            fx = 615.0
            fy = 615.0
            ppx = width / 2.0
            ppy = height / 2.0

        return {
            "depth_image_m": depth_image_m,
            "depth_image_raw": (depth_image_m / self.depth_scale_m).astype(np.uint16),
            "color_image_bgr": color_image_bgr,
            "intrinsics": DemoIntrinsics(),
        }
