# Veel gestelde vragen



## Kan ik de yolo-spatial-detector zonder RVIZ-monitor starten?
Start de applicatie als volgt:
```bash
ros2 launch my_depthai_python spatial_detector.launch.py rviz:=false
```

## Kan ik het camera URDF-model toevoegen aan mijn eigen robot URDF-model?
Start de camera-applicatie op zonder dat je RVIZ start, zie hierboven.
Start je "eigen" robot applicatie met daarin de eigen RVIZ visualisatie. 

Voeg in de RVIZ-configuratie een `RobotModel` toe met als `Description Topic` het `/camera_description` topic.

Voeg vervolgens in de RVIZ-configuratie topics toe die door de camera worden gegenereerd zoals bv.

```
Image: /color/image_rect
Image: /color/image_w_bouding_boxes
Marker: /color/ObjectText
PointCloud2: /stereo/points
```
:::{tip}
Vergeet niet naderhand de configuratie op te slaan
:::

## Kan ik de positie van de camera wijzigen?
Ja dat kan, pas in de juiste launchfile in de `launch` map van de `my_depthai_python` package onderstaande regels aan:


* `spatial_detector.launch.py`

```python

    tf_prefix_arg = DeclareLaunchArgument(
        'tf_prefix',
        default_value='oak',
        description='TF prefix / camera name for URDF frames',
    )
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='oak-d_frame',
        description='Base frame name for camera URDF',
    )
    parent_frame_arg = DeclareLaunchArgument(
        'parent_frame',
        default_value='world',
        description='Parent frame to attach camera URDF',
    )
    cam_pos_x_arg = DeclareLaunchArgument(
        'cam_pos_x',
        default_value='0.25',
        description='Camera X position relative to parent frame',
    )
    cam_pos_y_arg = DeclareLaunchArgument(
        'cam_pos_y',
        default_value='0.0',
        description='Camera Y position relative to parent frame',
    )
    cam_pos_z_arg = DeclareLaunchArgument(
        'cam_pos_z',
        default_value='0.5',
        description='Camera Z position relative to parent frame',
    )
    cam_roll_arg = DeclareLaunchArgument(
        'cam_roll',
        default_value='0.0',
        description='Camera roll relative to parent frame',
    )
    cam_pitch_arg = DeclareLaunchArgument(
        'cam_pitch',
        default_value='0.0',
        description='Camera pitch relative to parent frame',
    )
    cam_yaw_arg = DeclareLaunchArgument(
        'cam_yaw',
        default_value='0.0',
        description='Camera yaw relative to parent frame',
    )

```
De waarden zijn ten opzichte van het `world` frame, maar je kunt ook een referentie leggen naar de eigen robot, b.v. de end-effector van een robotarm. In dat geval dien je `parent_frame` aan te passen naar het frame van de robot waar je de camera aan wilt koppelen.

## Kan ik ander type camera, dan de `OAK-D` gebruiken
Pas in de juiste launchfile in de `launch` map van de `my_depthai_python` package onderstaande regels aan:

* `spatial_detector.launch.py`

```python
    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='OAK-D',
        description='DepthAI camera model used by the URDF',
    )
```
## Kan in het urdf-model van de camera ook combineren met mijn eigen robot model?
ja je kunt de camera URDF combineren met je eigen robot URDF. In dat geval dien je in de launchfile van de camera (bv. `spatial_detector.launch.py`) de `parent_frame` aan te passen naar het frame van de robot waar je de camera aan wilt koppelen.

je start dan de camera-applicatie op zonder dat je RVIZ start.
```bash
ros2 launch my_depthai_python spatial_detector.launch.py rviz:=false
```
Start je "eigen" robot applicatie met daarin de eigen RVIZ visualisatie. En voeg in de RVIZ-configuratie een `RobotModel` toe met als `Description Topic` het `/camera_description` topic.
Vergeet niet naderhand de configuratie op te slaan. Je kunt eveneens topics toevoegen die door de camera worden gegenereerd zoals bv.

```
/camera/rgb
/camera_description
```

als je de `pointcloud_from_images` node hebt gestart, dan kun je ook de volgende topics toevoegen:
```
/stereo/pointcloud
```


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