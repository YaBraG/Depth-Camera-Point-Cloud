DEPTH_WIDTH = 640
DEPTH_HEIGHT = 480
COLOR_WIDTH = 640
COLOR_HEIGHT = 480
FPS = 30

# Depth range
MIN_DEPTH_M = 0.20
MAX_DEPTH_M = 3.0

# Point density
POINT_STRIDE = 1
VOXEL_SIZE_M = 0.002

# RealSense depth filtering
ENABLE_DEPTH_FILTERS = True
ENABLE_THRESHOLD_FILTER = True
ENABLE_SPATIAL_FILTER = True
ENABLE_TEMPORAL_FILTER = True
ENABLE_HOLE_FILLING_FILTER = True

# Do not enable decimation yet because it changes image resolution and can break
# color/depth indexing.
ENABLE_DECIMATION_FILTER = False

# Optional filter tuning
SPATIAL_FILTER_SMOOTH_ALPHA = 0.5
SPATIAL_FILTER_SMOOTH_DELTA = 20
SPATIAL_FILTER_MAGNITUDE = 2

TEMPORAL_FILTER_SMOOTH_ALPHA = 0.4
TEMPORAL_FILTER_SMOOTH_DELTA = 20

HOLE_FILLING_MODE = 1

# 2D preview display correction only.
# These affect cv2.imshow previews only, not the raw depth math.
DISPLAY_ROTATE_180 = True
DISPLAY_MIRROR_HORIZONTAL = False
DISPLAY_MIRROR_VERTICAL = False

# 3D Open3D visualization correction only.
# These affect only points sent to Open3D, not the original projection equations.
POINTCLOUD_VISUAL_FLIP_X = True
POINTCLOUD_VISUAL_FLIP_Y = True
POINTCLOUD_VISUAL_FLIP_Z = False

# Debug and diagnostics
SHOW_DEPTH_DIAGNOSTICS = True
VERY_LOW_VALID_DEPTH_PIXEL_COUNT = 5000

# Mapping
USE_ICP = False
ICP_MAX_CORRESPONDENCE_DISTANCE_M = 0.05
AUTO_ACCUMULATE_EVERY_N_FRAMES = 30
AUTO_MAPPING_DEFAULT = False

# Viewer
SHOW_COORDINATE_FRAME = True
COORDINATE_FRAME_SIZE_M = 0.25

OUTPUT_DIR = "output"
DEMO_MODE = False
