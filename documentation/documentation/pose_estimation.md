# Pose Estimation

De package `my_depthai_pose_estimation` is een ROS 2 (Jazzy) pakket voor realtime hand-, mens- en dier-pose-estimatie met DepthAI OAK-camera's.

## Functies

- Handpose-pipeline met landmarks, marker-visualisatie en RViz-launch.
- Human-pose-pipeline met image output, landmarks output, marker-visualisatie en RViz-launch.
- Animal-pose-pipeline met image output, landmarks output, marker-visualisatie en RViz-launch.
- Reconnect-veilige startup als er geen OAK-device is aangesloten.

## Pakketstructuur

- `pose_estimation/hand_pose/` - handpose-nodes en assets
- `pose_estimation/human_pose/` - humaan-pose-nodes en assets
- `pose_estimation/animal_pose/` - dier-pose-nodes en assets
- `launch/` - launch-bestanden
- `rviz/` - RViz-presets

## Bouwen

Vanaf de root van de workspace:

```bash
cd ~/my_depthai_ws
colcon build --packages-select my_depthai_interfaces my_depthai_pose_estimation
source install/setup.bash
```

## Starten

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

## Gepubliceerde topics

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

## Nuttige launch-argumenten

Veelgebruikte argumenten in launch-bestanden:

- `device` - Optionele OAK device-id/naam/IP.
- `media_path` - Optioneel videobestand als input in plaats van camerastream.
- `fps_limit` - FPS-limiet tijdens runtime (`0` gebruikt platformstandaard).
- `image_topic` - Overschrijven van image output-topic.
- `landmarks_topic` - Overschrijven van landmarks output-topic (hand/human/animal).
- `markers_topic` - Overschrijven van marker-topic.
- `start_markers` - Marker-node starten (`true`/`false`).
- `start_rviz` - RViz starten (`true`/`false`).
- `rviz_config` - RViz-configpad overschrijven.
- `start_urdf` - Camera URDF/TF launch starten.

Voorbeeld:

```bash
ros2 launch my_depthai_pose_estimation human_pose.launch.py \
  start_rviz:=true \
  start_markers:=true \
  image_topic:=/demo/human/image
```

## Individuele nodes starten

```bash
ros2 run my_depthai_pose_estimation hand_pose_node
ros2 run my_depthai_pose_estimation hand_pose_markers_node
ros2 run my_depthai_pose_estimation human_pose_node
ros2 run my_depthai_pose_estimation human_pose_markers_node
ros2 run my_depthai_pose_estimation animal_pose_node
ros2 run my_depthai_pose_estimation animal_pose_markers_node
```

## Probleemoplossing

### Launch-fout: libexec-directory bestaat niet

Als launch meldt:

- `lib/my_depthai_pose_estimation does not exist`

Bouw opnieuw en source opnieuw:

```bash
cd ~/my_depthai_ws
colcon build --packages-select my_depthai_pose_estimation
source install/setup.bash
```

### Launch-fout: package not found

Als launch package-resolutieproblemen meldt, zorg voor een frisse omgeving:

```bash
cd ~/my_depthai_ws
source install/setup.bash
```

### Geen OAK-device aangesloten

Nodes proberen automatisch opnieuw via `reconnect_interval_sec` en starten zodra een device beschikbaar is.
