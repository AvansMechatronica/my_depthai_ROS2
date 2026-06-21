# my_depthai_object_tracking


ROS 2 (Jazzy) package for DepthAI-based person tracking with Kalman smoothing, image overlays, tracklet messages, and RViz markers.

## Features

- Kalman-smoothed object tracking pipeline using OAK RGB + stereo cameras.
- Reconnect-safe startup when no OAK device is connected.
- Annotated image publishing (`sensor_msgs/msg/Image`).
- Tracklet publishing (`my_depthai_interfaces/msg/TrackDetection2DArray`).
- Marker publishing (`visualization_msgs/msg/MarkerArray`) from tracklets.
- Optional RViz and optional camera URDF/TF launch.

## Package Layout

- `my_depthai_object_tracking/kalman_tracking/kalman_tracking_node.py` - main tracking node
- `my_depthai_object_tracking/kalman_tracking/kalman_tracking_markers_node.py` - marker node
- `my_depthai_object_tracking/kalman_tracking/depthai_models/` - model files (`.yaml` and `.tar.xz`)
- `launch/kalman_tracking.launch.py` - integrated launch file
- `rviz/kalman_tracking.rviz` - RViz preset

## Build

From workspace root:

```bash
cd ~/my_depthai_ws
colcon build --packages-select my_depthai_interfaces my_depthai_object_tracking
source install/setup.bash
```

## Run

### Full Launch (tracking + markers + RViz + optional URDF)

```bash
ros2 launch my_depthai_object_tracking kalman_tracking.launch.py
```

### Run Individual Nodes

```bash
ros2 run my_depthai_object_tracking kalman_tracking_node
ros2 run my_depthai_object_tracking kalman_tracking_markers_node
```

## Published Topics

- `/kalman_tracking/image` (`sensor_msgs/msg/Image`)
- `/kalman_tracking/tracklets` (`my_depthai_interfaces/msg/TrackDetection2DArray`)
- `/kalman_tracking/markers` (`visualization_msgs/msg/MarkerArray`)

## Useful Launch Arguments

- `device` - optional OAK device id/name/IP
- `fps_limit` - runtime FPS cap (`0` uses platform default)
- `model_name` - model file name located in `depthai_models/` (`.yaml` or `.tar.xz`)
- `model_path` - optional custom model file path (`.yaml` or `.tar.xz`), overrides `model_name` when set
- `image_topic` - image output topic
- `tracklets_topic` - tracklets output topic
- `markers_topic` - markers output topic
- `frame_id` - frame id for published image headers
- `reconnect_interval_sec` - retry period when device is unavailable
- `start_markers` - start marker node (`true`/`false`)
- `start_rviz` - start RViz (`true`/`false`)
- `rviz_config` - custom RViz config path
- `start_urdf` - start camera URDF/TF launch (`true`/`false`)
- `camera_model`, `tf_prefix`, `base_frame`, `parent_frame`, `cam_pos_x`, `cam_pos_y`, `cam_pos_z`, `cam_roll`, `cam_pitch`, `cam_yaw` - URDF/TF settings

Example:

```bash
ros2 launch my_depthai_object_tracking kalman_tracking.launch.py \
  start_rviz:=true \
  start_markers:=true \
  start_urdf:=true \
  model_name:=your_model_file.yaml \
  image_topic:=/demo/kalman/image \
  tracklets_topic:=/demo/kalman/tracklets \
  markers_topic:=/demo/kalman/markers
```

Use a custom model file:

```bash
ros2 launch my_depthai_object_tracking kalman_tracking.launch.py \
  model_path:=/absolute/path/to/your/model.rvc2.tar.xz
```

## Marker Node Parameters

- `tracklets_topic` (default: `/kalman_tracking/tracklets`)
- `markers_topic` (default: `/kalman_tracking/markers`)
- `point_scale` (default: `0.06`)
- `text_scale` (default: `0.08`)
- `marker_lifetime_sec` (default: `0.25`)

## Troubleshooting

### Device already in use (`X_LINK_DEVICE_ALREADY_IN_USE`)

Another process is using the OAK device. Stop stale processes and relaunch.

```bash
ps -ef | grep -Ei "kalman_tracking_node|ros2 launch my_depthai_object_tracking" | grep -v grep
pkill -f my_depthai_object_tracking/lib/my_depthai_object_tracking/kalman_tracking_node
```

Then restart launch.

### No OAK device connected

The node retries automatically every `reconnect_interval_sec` and starts when device becomes available.

### Package or executable not found

Rebuild and re-source:

```bash
cd ~/my_depthai_ws
colcon build --packages-select my_depthai_object_tracking
source install/setup.bash
```
