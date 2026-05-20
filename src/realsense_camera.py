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
        self.threshold_filter = None
        self.spatial_filter = None
        self.temporal_filter = None
        self.hole_filling_filter = None
        self.filter_warning = ""

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
        self._setup_depth_filters()
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
        self.threshold_filter = None
        self.spatial_filter = None
        self.temporal_filter = None
        self.hole_filling_filter = None
        self.is_streaming = False

    def _setup_depth_filters(self) -> None:
        self.filter_warning = ""
        self.threshold_filter = None
        self.spatial_filter = None
        self.temporal_filter = None
        self.hole_filling_filter = None
        if not config.ENABLE_DEPTH_FILTERS:
            return

        rs = self._import_realsense()
        warnings = []
        try:
            self.threshold_filter = rs.threshold_filter()
            self._safe_set_filter_option(self.threshold_filter, rs.option.min_distance, config.MIN_DEPTH_M)
            self._safe_set_filter_option(self.threshold_filter, rs.option.max_distance, config.MAX_DEPTH_M)
        except Exception as exc:
            warnings.append(f"threshold filter setup failed: {exc}")
            self.threshold_filter = None

        try:
            self.spatial_filter = rs.spatial_filter()
            self._safe_set_filter_option(self.spatial_filter, rs.option.filter_magnitude, config.SPATIAL_FILTER_MAGNITUDE)
            self._safe_set_filter_option(self.spatial_filter, rs.option.filter_smooth_alpha, config.SPATIAL_FILTER_SMOOTH_ALPHA)
            self._safe_set_filter_option(self.spatial_filter, rs.option.filter_smooth_delta, config.SPATIAL_FILTER_SMOOTH_DELTA)
        except Exception as exc:
            warnings.append(f"spatial filter setup failed: {exc}")
            self.spatial_filter = None

        try:
            self.temporal_filter = rs.temporal_filter()
            self._safe_set_filter_option(self.temporal_filter, rs.option.filter_smooth_alpha, config.TEMPORAL_FILTER_SMOOTH_ALPHA)
            self._safe_set_filter_option(self.temporal_filter, rs.option.filter_smooth_delta, config.TEMPORAL_FILTER_SMOOTH_DELTA)
        except Exception as exc:
            warnings.append(f"temporal filter setup failed: {exc}")
            self.temporal_filter = None

        try:
            self.hole_filling_filter = rs.hole_filling_filter()
            self._safe_set_filter_option(self.hole_filling_filter, rs.option.holes_fill, config.HOLE_FILLING_MODE)
        except Exception as exc:
            warnings.append(f"hole filling filter setup failed: {exc}")
            self.hole_filling_filter = None

        option_warnings = [self.filter_warning] if self.filter_warning else []
        self.filter_warning = "; ".join(option_warnings + warnings)

    def _safe_set_filter_option(self, filter_obj, option, value) -> None:
        try:
            if filter_obj.supports(option):
                filter_obj.set_option(option, float(value))
        except Exception as exc:
            message = f"filter option warning: {exc}"
            self.filter_warning = f"{self.filter_warning}; {message}" if self.filter_warning else message

    def _apply_depth_filters(self, depth_frame):
        if not config.ENABLE_DEPTH_FILTERS:
            return depth_frame

        filtered_depth_frame = depth_frame
        warnings = []
        filter_steps = [
            (config.ENABLE_THRESHOLD_FILTER, self.threshold_filter, "threshold"),
            (config.ENABLE_SPATIAL_FILTER, self.spatial_filter, "spatial"),
            (config.ENABLE_TEMPORAL_FILTER, self.temporal_filter, "temporal"),
            (config.ENABLE_HOLE_FILLING_FILTER, self.hole_filling_filter, "hole filling"),
        ]
        for enabled, filter_obj, name in filter_steps:
            if not enabled or filter_obj is None:
                continue
            try:
                filtered_depth_frame = filter_obj.process(filtered_depth_frame)
            except Exception as exc:
                warnings.append(f"{name} filter failed: {exc}")

        if warnings:
            existing_warning = [self.filter_warning] if self.filter_warning else []
            self.filter_warning = "; ".join(existing_warning + warnings)
        return filtered_depth_frame

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

        filtered_depth_frame = self._apply_depth_filters(depth_frame)
        depth_image_raw = np.asanyarray(filtered_depth_frame.get_data())
        color_image_bgr = np.asanyarray(color_frame.get_data())
        depth_image_m = depth_image_raw.astype(np.float32) * self.depth_scale_m
        try:
            intrinsics = filtered_depth_frame.profile.as_video_stream_profile().intrinsics
        except Exception:
            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        depth_stats = compute_depth_stats(depth_image_m)

        return {
            "depth_image_m": depth_image_m,
            "depth_image_raw": depth_image_raw,
            "color_image_bgr": color_image_bgr,
            "intrinsics": intrinsics,
            "filter_warning": self.filter_warning,
            "depth_stats": depth_stats,
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
            "filter_warning": "",
            "depth_stats": compute_depth_stats(depth_image_m),
        }


def compute_depth_stats(depth_image_m):
    valid_depth_mask = np.isfinite(depth_image_m)
    valid_depth_mask &= depth_image_m >= config.MIN_DEPTH_M
    valid_depth_mask &= depth_image_m <= config.MAX_DEPTH_M
    valid_depth_values_m = depth_image_m[valid_depth_mask]
    valid_count = int(valid_depth_values_m.size)

    if valid_count == 0:
        return {
            "valid_count": 0,
            "depth_min_m": None,
            "depth_median_m": None,
            "depth_max_m": None,
        }

    return {
        "valid_count": valid_count,
        "depth_min_m": float(np.min(valid_depth_values_m)),
        "depth_median_m": float(np.median(valid_depth_values_m)),
        "depth_max_m": float(np.max(valid_depth_values_m)),
    }
