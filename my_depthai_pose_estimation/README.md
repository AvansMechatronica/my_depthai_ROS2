# my_depthai_pose_estimation

ROS 2 (Jazzy) package for real-time hand, human, and animal pose estimation using DepthAI OAK cameras.

## Features

- Hand pose pipeline with landmarks, marker visualization, and RViz launch.
- Human pose pipeline with image output, landmarks output, marker visualization, and RViz launch.
- Animal pose pipeline with image output, landmarks output, marker visualization, and RViz launch.
- Reconnect-safe startup behavior when no OAK device is connected.

## Package Layout

- `pose_estimation/hand_pose/` - hand pose nodes and assets
- `pose_estimation/human_pose/` - human pose nodes and assets
- `pose_estimation/animal_pose/` - animal pose nodes and assets
- `launch/` - launch files
- `rviz/` - RViz presets

## Build

From the workspace root:

```bash
cd ~/my_depthai_ws
colcon build --packages-select my_depthai_interfaces my_depthai_pose_estimation
source install/setup.bash
```

## Run

### Hand Pose

```bash
ros2 launch my_depthai_pose_estimation hand_pose.launch.py
```

### Human Pose

```bash
ros2 launch my_depthai_pose_estimation human_pose.launch.py
```

### Animal Pose

```bash
ros2 launch my_depthai_pose_estimation animal_pose.launch.py
```

## Published Topics

### Hand

- `/hand_pose/image` (`sensor_msgs/msg/Image`)
- `/hand_pose/landmarks` (`my_depthai_interfaces/msg/HandLandmarkArray`)
- `/hand_pose/markers` (`visualization_msgs/msg/MarkerArray`)

### Human

- `/human_pose/image` (`sensor_msgs/msg/Image`)
- `/human_pose/landmarks` (`my_depthai_interfaces/msg/HumanLandmarkArray`)
- `/human_pose/markers` (`visualization_msgs/msg/MarkerArray`)

### Animal

- `/animal_pose/image` (`sensor_msgs/msg/Image`)
- `/animal_pose/landmarks` (`my_depthai_interfaces/msg/AnimalLandmarkArray`)
- `/animal_pose/markers` (`visualization_msgs/msg/MarkerArray`)

## Useful Launch Arguments

Common arguments available in launch files:

- `device` - Optional OAK device id/name/IP.
- `media_path` - Optional video file input instead of camera stream.
- `fps_limit` - Runtime FPS cap (`0` uses platform default).
- `image_topic` - Output image topic override.
- `landmarks_topic` - Output landmarks topic override (human/animal/hand where applicable).
- `markers_topic` - Marker topic override.
- `start_markers` - Start marker node (`true`/`false`).
- `start_rviz` - Start RViz (`true`/`false`).
- `rviz_config` - RViz config path override.
- `start_urdf` - Start camera URDF/TF launch.

Example:

```bash
ros2 launch my_depthai_pose_estimation human_pose.launch.py \
  start_rviz:=true \
  start_markers:=true \
  image_topic:=/demo/human/image
```

## Run Individual Nodes

```bash
ros2 run my_depthai_pose_estimation hand_pose_node
ros2 run my_depthai_pose_estimation hand_pose_markers_node
ros2 run my_depthai_pose_estimation human_pose_node
ros2 run my_depthai_pose_estimation human_pose_markers_node
ros2 run my_depthai_pose_estimation animal_pose_node
ros2 run my_depthai_pose_estimation animal_pose_markers_node
```

## Troubleshooting

### Launch error: libexec directory does not exist

If launch reports:

- `lib/my_depthai_pose_estimation does not exist`

Rebuild and re-source:

```bash
cd ~/my_depthai_ws
colcon build --packages-select my_depthai_pose_estimation
source install/setup.bash
```

### Launch error: package not found

If launch reports package resolution issues, ensure your environment is fresh:

```bash
cd ~/my_depthai_ws
source install/setup.bash
```

### No OAK device connected

Nodes retry automatically using `reconnect_interval_sec` and will start once a device is available.
