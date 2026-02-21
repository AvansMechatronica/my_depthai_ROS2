# Veel gestelde vragen

## Kan ik de circel-detector zonder RVIZ-monit als volgt op:
Ja dat kan, start de applicatie als volgt:
```bash
ros2 launch my_depthai stereo_circle_detector.launch.py rviz:=false
```

## Kan ik de yolo-spatial-detector zonder RVIZ-monit als volgt op:
Ja dat kan,start de applicatie als volgt:
```bash
ros2 launch my_depthai yolo_spatial_detector_node.launch.py rviz:=false
```



## Kan ik de positie van de camera wijzigen
Ja dat kan, pas in de juiste launchfile in de `launch` map van de `my_depthai` package onderstaande regels aan:

* `stereo_circle_detector.launch.py`
* `yolo_spatial_detector_node.launch.py`

```python
cam_pos_x = LaunchConfiguration('cam_pos_x',     default = '0.25')
cam_pos_y = LaunchConfiguration('cam_pos_y',     default = '0.0')
cam_pos_z = LaunchConfiguration('cam_pos_z',     default = '0.5')
cam_roll  = LaunchConfiguration('cam_roll',      default = '0.0')
cam_pitch = LaunchConfiguration('cam_pitch',     default = '0.0')
cam_yaw   = LaunchConfiguration('cam_yaw',       default = '0.0')
```

## Kan ik ander type camera, dan de `OAK-D` gebruiken
Ja dat kan, pas in de juiste launchfile in de `launch` map van de `my_depthai` package onderstaande regels aan:

* `stereo_circle_detector.launch.py`
* `yolo_spatial_detector_node.launch.py`

```python
camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')
```
## Kan in het urdf model ook
## Wanneer moet ik `colcon build` gebruiken?
Het `colcon build --symlink-install` wordt alleen gebruikt voor de volgende situaties:
* Er zijn bestanden aan een ROS2 package toegevoegd
* Er is een nieuwe ROS2 package gemaakt
* De inhoud van een `C` of `C++` bestand is gewijzigd
* De inhoud van de setup.py van een Python package is gewijzigd
* De inhoud van `package.xml` of `CMakeLists.txt` is gewijzigd

Na de build dien je altijd het volgende commando in `alle openstaande` terminals uit te voeren:
```
source ~/my_ur_ws/install/setup.bash
```