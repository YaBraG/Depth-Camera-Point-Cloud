from __future__ import annotations

from datetime import datetime
from pathlib import Path

from . import config


def ensure_output_dir() -> Path:
    output_dir = Path(config.OUTPUT_DIR)
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def timestamped_ply_path(prefix: str) -> Path:
    ensure_output_dir()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(config.OUTPUT_DIR) / f"{prefix}_{timestamp}.ply"


def save_open3d_point_cloud(point_cloud, prefix: str) -> Path:
    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError("Open3D is not installed. Run: py -3.12 -m pip install open3d") from exc

    output_path = timestamped_ply_path(prefix)
    if point_cloud is None or len(point_cloud.points) == 0:
        raise RuntimeError("No point cloud points are available to save.")

    success = o3d.io.write_point_cloud(str(output_path), point_cloud, write_ascii=False)
    if not success:
        raise RuntimeError(f"Open3D could not save point cloud to {output_path}")
    return output_path
