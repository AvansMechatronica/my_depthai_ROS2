# Object Tracking
De package `my_depthai_object_tracking` is een ROS 2 package voor DepthAI-gebaseerde persoons-tracking met Kalman-smoothing, beeld-annotaties, tracklet-berichten en RViz-markers.


## Functionaliteit

- Kalman-gesmoothde objecttracking-pipeline met OAK RGB + stereo camera's.
- Veilige herstartlogica wanneer geen OAK-device is aangesloten.
- Publicatie van geannoteerde beelden (`sensor_msgs/msg/Image`).
- Publicatie van tracklets (`my_depthai_interfaces/msg/TrackDetection2DArray`).
- Publicatie van markers (`visualization_msgs/msg/MarkerArray`) op basis van tracklets.
- Optionele RViz-start en optionele camera URDF/TF launch.

## Pakketstructuur

- `my_depthai_object_tracking/kalman_tracking/kalman_tracking_node.py` - hoofdnode voor tracking
- `my_depthai_object_tracking/kalman_tracking/kalman_tracking_markers_node.py` - markernode
- `my_depthai_object_tracking/kalman_tracking/depthai_models/` - modelbestanden (`.yaml` en `.tar.xz`)
- `launch/kalman_tracking.launch.py` - geïntegreerde launchfile
- `rviz/kalman_tracking.rviz` - RViz preset

## Build

Vanuit de workspace-root:

```bash
cd ~/my_depthai_ws
colcon build --packages-select my_depthai_interfaces my_depthai_object_tracking
source install/setup.bash
```

## Starten

### Volledige launch (tracking + markers + RViz + optionele URDF)

```bash
ros2 launch my_depthai_object_tracking kalman_tracking.launch.py
```

### Losse nodes starten

```bash
ros2 run my_depthai_object_tracking kalman_tracking_node
ros2 run my_depthai_object_tracking kalman_tracking_markers_node
```

## Gepubliceerde topics

- `/kalman_tracking/image` (`sensor_msgs/msg/Image`)
- `/kalman_tracking/tracklets` (`my_depthai_interfaces/msg/TrackDetection2DArray`)
- `/kalman_tracking/markers` (`visualization_msgs/msg/MarkerArray`)

## Handige launch-argumenten

- `device` - optionele OAK device id/naam/IP
- `fps_limit` - runtime FPS-limiet (`0` gebruikt platform-default)
- `model_name` - preset model-bestandsnaam (bijvoorbeeld `yolov6_nano_r2_coco.RVC2.yaml` of `SimpleFruitsYoloV8.rvc2.tar.xz`)
- `model_path` - optioneel pad naar custom modelbestand (`.yaml` of `.tar.xz`), overschrijft `model_name` als gezet
- `image_topic` - output topic voor beeld
- `tracklets_topic` - output topic voor tracklets
- `markers_topic` - output topic voor markers
- `frame_id` - frame id voor gepubliceerde image headers
- `reconnect_interval_sec` - retry-interval als device niet beschikbaar is
- `start_markers` - markernode starten (`true`/`false`)
- `start_rviz` - RViz starten (`true`/`false`)
- `rviz_config` - pad naar custom RViz-config
- `start_urdf` - camera URDF/TF launch starten (`true`/`false`)
- `camera_model`, `tf_prefix`, `base_frame`, `parent_frame`, `cam_pos_x`, `cam_pos_y`, `cam_pos_z`, `cam_roll`, `cam_pitch`, `cam_yaw` - URDF/TF instellingen

Voorbeeld:

```bash
ros2 launch my_depthai_object_tracking kalman_tracking.launch.py \
  start_rviz:=true \
  start_markers:=true \
  start_urdf:=true \
  model_name:=SimpleFruitsYoloV8.rvc2.tar.xz \
  image_topic:=/demo/kalman/image \
  tracklets_topic:=/demo/kalman/tracklets \
  markers_topic:=/demo/kalman/markers
```

Gebruik een custom modelbestand:

```bash
ros2 launch my_depthai_object_tracking kalman_tracking.launch.py \
  model_path:=/absoluut/pad/naar/je/model.rvc2.tar.xz
```

## Parameters van de markernode

- `tracklets_topic` (default: `/kalman_tracking/tracklets`)
- `markers_topic` (default: `/kalman_tracking/markers`)
- `point_scale` (default: `0.06`)
- `text_scale` (default: `0.08`)
- `marker_lifetime_sec` (default: `0.25`)

## Probleemoplossing

### Device al in gebruik (`X_LINK_DEVICE_ALREADY_IN_USE`)

Een ander proces gebruikt het OAK-device. Stop oude processen en start opnieuw.

```bash
ps -ef | grep -Ei "kalman_tracking_node|ros2 launch my_depthai_object_tracking" | grep -v grep
pkill -f my_depthai_object_tracking/lib/my_depthai_object_tracking/kalman_tracking_node
```

Start daarna opnieuw.

### Geen OAK-device aangesloten

De node probeert automatisch opnieuw volgens `reconnect_interval_sec` en start zodra een device beschikbaar is.

### Pakket of executable niet gevonden

Opnieuw builden en sourcen:

```bash
cd ~/my_depthai_ws
colcon build --packages-select my_depthai_object_tracking
source install/setup.bash
```
