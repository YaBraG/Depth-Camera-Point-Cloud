# RealSense D435 3D Mapper

This is a beginner-friendly Python 3.12 Windows prototype for testing Intel RealSense D435 RGB-D camera mapping ideas from one app. It opens a Tkinter control window, uses OpenCV for RGB/depth previews, and uses Open3D for point cloud viewing and `.ply` export.

Run everything from:

```bat
py -3.12 main.py
```

## D435 vs True 3D LiDAR

The Intel RealSense D435 is an RGB-D stereo depth camera. It estimates depth from stereo infrared images and projects depth pixels into 3D points. A true 3D LiDAR directly measures distance with laser scanning or time-of-flight over a wider range. The D435 is great for close-range indoor experiments, but it is more sensitive to lighting, reflective surfaces, texture, range, and USB bandwidth.

This project is a simple point-cloud mapper, not full SLAM. By default, accumulated frames are added in the same camera frame. Optional ICP can be enabled in `src/config.py`, but it is off by default.

## Install

Double-click `install.bat`, or run:

```bat
py -3.12 -m pip install --upgrade pip
py -3.12 -m pip install -r requirements.txt
```

## Run

```bat
py -3.12 main.py
```

## Buttons

- **Test Camera**: checks whether a RealSense camera is detected.
- **Show RGB**: opens a live OpenCV color preview.
- **Show Depth**: opens a colorized depth preview.
- **Show Aligned RGB-D**: shows RGB next to depth aligned to the color stream.
- **Show Live Point Cloud**: opens an Open3D live point cloud viewer.
- **Add Frame to Map**: adds the current point cloud to the accumulated map.
- **Toggle Auto Mapping**: automatically adds every Nth point cloud frame.
- **Save Current Point Cloud**: saves the current cloud to `output/current_cloud_*.ply`.
- **Save Full Map**: saves the accumulated map to `output/accumulated_map_*.ply`.
- **Clear Map**: clears the accumulated map.
- **Stop Stream**: stops previews and the RealSense pipeline.
- **Exit**: safely closes all windows and stops the camera.

## Keyboard Shortcuts

- `q` or `ESC`: stop current stream / close preview
- `s`: save current point cloud
- `m`: add current frame to accumulated map
- `a`: toggle automatic mapping
- `c`: clear map
- `p`: save accumulated map
- `h`: print help

## Troubleshooting

### Camera Not Detected

Check that the D435 is plugged in, then test it in Intel RealSense Viewer. Try another USB port and cable.

### pyrealsense2 Install Failure

Run:

```bat
py -3.12 -m pip install pyrealsense2
```

If no compatible wheel exists for your exact Python version, install a supported Python version or RealSense SDK package that matches your environment.

### USB 2 vs USB 3 Issue

The D435 works best on USB 3. USB 2 may reduce frame rate, resolution, or stream reliability. Use a USB 3 port and a known-good USB 3 cable.

### No Depth Frame

Stop the stream and start it again. Close Intel RealSense Viewer or any other app that may already be using the camera.

### Open3D Viewer Not Opening

Install Open3D:

```bat
py -3.12 -m pip install open3d
```

Also make sure your graphics drivers are current.

### App Freezes

Use **Stop Stream** or press `q`/`ESC`. The app uses Tkinter `.after()` updates instead of long blocking loops, but slow machines may need a larger `POINT_STRIDE` or `VOXEL_SIZE_M` in `src/config.py`.

## ROS 2 Migration Plan

The code is split so `pointcloud_math.py` and `map_builder.py` can be reused later in a ROS 2 package. A future ROS 2 node could publish:

- `sensor_msgs/PointCloud2`
- RGB image
- depth image
- camera intrinsics

See `src/ros2_notes.md` for more detail.
