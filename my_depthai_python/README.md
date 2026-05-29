# my_depthai_python

ROS 2 Python package that bridges [DepthAI](https://docs.luxonis.com/) OAK cameras to ROS 2 topics.  
It provides a minimal RGB publisher, a spatial object-detection node (YOLOv8 / YOLOv5), and two utility nodes for point-cloud generation and detection-based TF publishing.

## Table of contents

- [Requirements](#requirements)
- [Build](#build)
- [Package structure](#package-structure)
- [Nodes](#nodes)
  - [depthai_template](#depthai_template)
  - [spatial_detector](#spatial_detector)
  - [pointcloud_from_images](#pointcloud_from_images)
  - [publisch_tf](#publisch_tf)
- [Launch files](#launch-files)
- [Configuration files](#configuration-files)
- [Network assets](#network-assets)

---

## Requirements

| Dependency | Notes |
|---|---|
| ROS 2 Jazzy (or later) | `rclpy`, `sensor_msgs`, `geometry_msgs`, `vision_msgs` |
| `depthai >= 3.6.0` | Install in the same Python environment used for `colcon build` |
| `cv_bridge` | ROS ↔ OpenCV image conversion |
| `depthai_ros_msgs` | `SpatialDetection` / `SpatialDetectionArray` message types |
| `message_filters` | Time-synchronisation for `pointcloud_from_images` |

> **Important:** `depthai` must be installed for the same Python interpreter that `colcon build` and `ros2 run` use.  
> The recommended setup is a workspace virtualenv:
> ```bash
> python3 -m venv .venv --system-site-packages
> source .venv/bin/activate
> pip install depthai
> colcon build
> ```

---

## Build

```bash
cd ~/my_depthai_ws
source /opt/ros/jazzy/setup.bash
source src/my_depthai_ROS2/.venv/bin/activate   # activate the venv that has depthai
colcon build --packages-select my_depthai_python
source install/setup.bash
```

---

## Package structure

```
my_depthai_python/
├── config/
│   ├── depthai_template.yaml          # Parameters for depthai_template node
│   ├── pointcloud_from_images.yaml    # Parameters for pointcloud_from_images node
│   ├── publish_tf.yaml                # Parameters for publisch_tf node
│   └── spatial_detector.yaml         # Parameters for spatial_detector node
├── launch/
│   ├── depthai_template.launch.py
│   ├── pointcloud_from_images.launch.py
│   ├── publish_tf.launch.py
│   └── spatial_detector.launch.py
├── my_depthai_python/
│   ├── depthai_template.py            # RGB publisher node
│   ├── spatial_detector.py            # Spatial detection node
│   └── tools/
│       ├── pointcloud_from_images.py  # Point-cloud utility node
│       └── publisch_tf.py             # Detection → TF / marker node
├── resources/
│   └── *.rvc2.tar.xz / *.blob / *.json  # Bundled NN archives
└── rviz/
    ├── depthai_template.rviz
    └── spatial_detector.rviz
```

---

## Nodes

### depthai_template

Minimal node that opens the OAK camera RGB sensor and publishes frames as `sensor_msgs/Image` together with the corresponding `sensor_msgs/CameraInfo` (intrinsics read from device calibration).

**Publishers**

| Topic | Type | Description |
|---|---|---|
| `camera/rgb` *(default)* | `sensor_msgs/Image` | Raw RGB frames |
| `camera/camera_info` *(default)* | `sensor_msgs/CameraInfo` | Camera intrinsics |

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `topic_name` | `camera/rgb` | Output image topic |
| `camera_info_topic` | `camera/camera_info` | Output camera-info topic |
| `width` | `640` | Output image width (pixels) |
| `height` | `400` | Output image height (pixels) |
| `fps` | `30.0` | Camera frame rate |
| `queue_size` | `4` | DepthAI output queue size |

**Run**

```bash
ros2 run my_depthai_python depthai_template
```

---

### spatial_detector

Full spatial-detection pipeline using a YOLOv8 (or YOLOv5) NNArchive.  
Publishes detected objects with 3-D positions, the RGB image with bounding boxes drawn, and a depth image.  
Includes automatic reconnect logic with optional FPS degradation on repeated reconnects.

**Publishers**

| Topic | Type | Description |
|---|---|---|
| `spatial_detections` | `depthai_ros_msgs/SpatialDetectionArray` | Detected objects with 3-D positions |
| `camera/rgb` | `sensor_msgs/Image` | RGB image (optionally with bounding boxes) |
| `camera/camera_info` | `sensor_msgs/CameraInfo` | RGB camera intrinsics |
| `stereo/depth` | `sensor_msgs/Image` | Depth image (32FC1 metres) |
| `stereo/depth_raw` | `sensor_msgs/Image` | Raw depth image (mono16 millimetres) |

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `image_topic` | `camera/rgb` | RGB output topic |
| `depth_topic` | `stereo/depth` | Depth output topic (float32, metres) |
| `depth_raw_topic` | `stereo/depth_raw` | Raw depth output topic (mono16, mm) |
| `camera_info_topic` | `camera/camera_info` | CameraInfo output topic |
| `detections_topic` | `spatial_detections` | Detection output topic |
| `width` | `640` | Camera resolution width |
| `height` | `480` | Camera resolution height |
| `fps` | `8.0` | Camera / detector frame rate |
| `queue_size` | `1` | DepthAI output queue size |
| `nn_archive` | `SimpleFruitsYoloV8.rvc2.tar.xz` | NNArchive filename (looked up in `resources/`) |
| `show_bounding_boxes` | `true` | Draw bounding boxes on the published image |
| `publish_images` | `true` | Publish RGB and depth images |
| `depth_source` | `stereo` | Depth source (`stereo`) |
| `stereo_extended_disparity` | `false` | Enable extended disparity mode |
| `spatial_calc_algorithm` | `MIN` | Spatial location calculator algorithm |
| `reconnect_cooldown_sec` | `2.0` | Seconds to wait between reconnect attempts |
| `max_reconnect_attempts` | `0` | Max reconnects before giving up (0 = unlimited) |
| `auto_degrade_on_reconnect` | `true` | Reduce FPS after repeated reconnects |
| `reconnect_degrade_threshold` | `2` | Number of reconnects before FPS degradation |
| `degraded_fps` | `3.0` | FPS to use after degradation |
| `data_stall_timeout_sec` | `5.0` | Seconds without data before triggering reconnect |

**Run**

```bash
ros2 run my_depthai_python spatial_detector
```

---

### pointcloud_from_images

Utility node that subscribes to a depth image and an RGB image, time-synchronises them, and publishes a coloured `sensor_msgs/PointCloud2`.  
Camera intrinsics are taken either from a `CameraInfo` topic or from explicit `fx/fy/cx/cy` parameters.

**Subscribers**

| Topic | Type | Description |
|---|---|---|
| `stereo/depth_raw` | `sensor_msgs/Image` | Input depth image (mono16, mm) |
| `camera/rgb` | `sensor_msgs/Image` | Input RGB image |
| `camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |

**Publishers**

| Topic | Type | Description |
|---|---|---|
| `stereo/pointcloud` | `sensor_msgs/PointCloud2` | Coloured point cloud |

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `depth_topic` | `stereo/depth` | Input depth topic |
| `rgb_topic` | `camera/rgb` | Input RGB topic |
| `camera_info_topic` | `camera/camera_info` | Input CameraInfo topic |
| `pointcloud_topic` | `stereo/pointcloud` | Output PointCloud2 topic |
| `depth_scale` | `0.001` | Scale factor depth → metres |
| `min_depth_m` | `0.2` | Minimum depth to include (metres) |
| `max_depth_m` | `10.0` | Maximum depth to include (metres) |
| `pixel_step` | `2` | Subsample every N pixels (1 = full resolution) |
| `resize_rgb_to_depth` | `true` | Resize RGB to match depth resolution |
| `use_color` | `true` | Produce coloured (XYZRGB) or XYZ-only cloud |
| `sync_queue_size` | `10` | `message_filters` queue size |
| `sync_slop_sec` | `0.05` | Approximate-time sync tolerance (seconds) |
| `fx`, `fy`, `cx`, `cy` | `0.0` | Override intrinsics (used when CameraInfo is unavailable) |

**Run**

```bash
ros2 run my_depthai_python pointcloud_from_images
```

---

### publisch_tf

Utility node that listens to `SpatialDetectionArray` messages and re-publishes each detected object as a TF transform and a `visualization_msgs/Marker` (text label) in RViz.  
Class labels are read automatically from the NNArchive `config.json`.

**Subscribers**

| Topic | Type | Description |
|---|---|---|
| `spatial_detections` *(default)* | `depthai_ros_msgs/SpatialDetectionArray` | Input detections |

**Publishers**

| Topic | Type | Description |
|---|---|---|
| `color/ObjectText` *(default)* | `visualization_msgs/MarkerArray` | Text markers per detection |
| `/tf` | `tf2_msgs/TFMessage` | TF transform per detection |

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `nn_archive` | `""` | NNArchive filename for label extraction |
| `detections_topic` | `color/yolov4_spatial_detections` | Input detections topic |
| `marker_topic` | `color/ObjectText` | Output marker topic |
| `frame_id` | `oak_rgb_camera_optical_frame` | TF parent frame |
| `marker_lifetime_sec` | `10.0` | Marker display duration (seconds) |

**Run**

```bash
ros2 run my_depthai_python publisch_tf
```

---

## Launch files

### depthai_template.launch.py

Starts the `depthai_template` node and optionally RViz2.

```bash
ros2 launch my_depthai_python depthai_template.launch.py
```

Key arguments:

| Argument | Default | Description |
|---|---|---|
| `topic_name` | `camera/rgb` | RGB output topic |
| `width` / `height` | `640` / `400` | Output resolution |
| `fps` | `30.0` | Frame rate |
| `start_rviz` | `true` | Launch RViz2 |
| `params_file` | `config/depthai_template.yaml` | YAML parameter override |

---

### spatial_detector.launch.py

Starts the `spatial_detector` node and optionally RViz2 and the camera URDF/TF publisher.

```bash
ros2 launch my_depthai_python spatial_detector.launch.py
```

Key arguments:

| Argument | Default | Description |
|---|---|---|
| `start_rviz` | `true` | Launch RViz2 |
| `start_urdf` | `true` | Launch camera URDF / TF publisher |
| `camera_model` | `OAK-D` | Camera model for URDF |
| `tf_prefix` | `oak` | TF prefix for camera frames |
| `parent_frame` | `world` | Frame to attach camera to |
| `cam_pos_x/y/z` | `0.25 / 0.0 / 0.5` | Camera position (metres) |
| `cam_roll/pitch/yaw` | `0.0` | Camera orientation (radians) |
| `rviz_config` | `rviz/spatial_detector.rviz` | RViz2 config path |

---

### pointcloud_from_images.launch.py

Starts the `pointcloud_from_images` node.

```bash
ros2 launch my_depthai_python pointcloud_from_images.launch.py
```

Key arguments:

| Argument | Default | Description |
|---|---|---|
| `depth_topic` | `stereo/depth_raw` | Input depth topic |
| `rgb_topic` | `camera/rgb` | Input RGB topic |
| `camera_info_topic` | `camera/camera_info` | Input CameraInfo topic |
| `pointcloud_topic` | `stereo/pointcloud` | Output PointCloud2 topic |

---

### publish_tf.launch.py

Starts the `publisch_tf` node using the settings from `config/publish_tf.yaml`.

```bash
ros2 launch my_depthai_python publish_tf.launch.py
```

---

## Configuration files

All YAML files live in `config/` and are installed to `share/my_depthai_python/config/`.  
Each file maps to a node via its top-level key (e.g. `spatial_detector: ros__parameters: ...`).  
Pass a custom file at launch time with the `params_file` argument.

---

## Network assets

Bundled model archives are located in `resources/` and installed to `share/my_depthai_python/resources/`.

| File | Description |
|---|---|
| `SimpleFruitsYoloV8.rvc2.tar.xz` | YOLOv8 fruit detector (default for `spatial_detector`) |

To use a custom model, place the `.rvc2.tar.xz` NNArchive in `resources/`, rebuild the package, and set the `nn_archive` parameter to the filename.
