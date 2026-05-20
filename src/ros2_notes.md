# ROS 2 Portability Notes

Suggested future package name:

`realsense_3d_mapper`

## Files Designed For Reuse

- `pointcloud_math.py`: reusable RGB-D projection math. A ROS 2 node can pass a depth image, RGB image, and camera intrinsics into `create_point_cloud_from_rgbd()`.
- `map_builder.py`: reusable accumulated point cloud logic. The same class can add, downsample, clear, and save map clouds.
- `config.py`: reusable constants for image size, depth range, stride, voxel size, and ICP settings.
- `file_io.py`: reusable `.ply` save helper if the ROS 2 package keeps Open3D as a dependency.

## File That Would Change Most

- `realsense_camera.py` is a direct RealSense pipeline wrapper. In ROS 2, this may be replaced by subscriptions to camera topics or by a ROS 2 node that owns the RealSense device.
- `app.py` is a desktop Tkinter/OpenCV/Open3D test harness. It is useful for local testing, but it would not become the ROS 2 runtime node.

## Future ROS 2 Node Shape

A future node could be named:

`realsense_3d_mapper_node`

It would publish:

- `/camera/color/image_raw` as `sensor_msgs/msg/Image`
- `/camera/depth/image_rect_raw` as `sensor_msgs/msg/Image`
- `/camera/color/camera_info` as `sensor_msgs/msg/CameraInfo`
- `/mapper/points` as `sensor_msgs/msg/PointCloud2`
- `/mapper/accumulated_map` as `sensor_msgs/msg/PointCloud2`

It could expose services:

- `/mapper/add_frame`
- `/mapper/clear_map`
- `/mapper/save_map`
- `/mapper/toggle_auto_mapping`

## Migration Plan

1. Create a ROS 2 Python package named `realsense_3d_mapper`.
2. Move or import `pointcloud_math.py`, `map_builder.py`, and `config.py`.
3. Create a ROS 2 node class that receives RGB, depth, and camera info messages.
4. Convert ROS image messages to NumPy arrays using `cv_bridge`.
5. Reuse `create_point_cloud_from_rgbd()` to build NumPy point arrays.
6. Convert NumPy points/colors to `sensor_msgs/PointCloud2`.
7. Reuse `MapBuilder` for the accumulated map.

ICP can stay optional. It estimates a rigid transform between point clouds, but it is not a replacement for full SLAM, loop closure, or reliable camera odometry.
