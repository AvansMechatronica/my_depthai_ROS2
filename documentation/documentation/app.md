# Applicaties


## Cirkel detector
Deze applicatie demonstreet een cirkel detect in een afbeelding en publiceert de gedetecteerd circles in een nieuwe afbeelding.

```bash
ros2 launch my_depthai stereo_circle_detector.launch.py
```
![image](../images/circle_detector.png)


## Yolo spatial(ruimtelijk) network detector
Deze appilicatie  demonsteert de cassificatie en positie van een object gedetecteerd met een neuraal netwerk. De positie van het object is relatief t.o.v de camera.

```bash
ros2 launch my_depthai yolo_spatial_detector_node.launch.py
```
![image](../images/camera_opstelling.png)
![image](../images/nn_objects.jpg)


## Python template (DepthAI)
Deze applicatie publiceert RGB-beelden van een OAK-camera op een ROS2 topic.

```bash
ros2 run my_depthai_python_template depthai_template
```

### Stabiele installatie (zonder system-pip)
Gebruik een virtual environment en bouw de package in diezelfde environment. Dan krijgt de gegenereerde `ros2 run` entrypoint automatisch de juiste Python interpreter.

```bash
cd ~/my_depthai_ws/src/my_depthai_ROS2
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip

# DepthAI python dependency
python -m pip install depthai

cd ~/my_depthai_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_depthai_python_template
source install/setup.bash
ros2 run my_depthai_python_template depthai_template
```

### Optionele parameters
- `topic_name` (default: `camera/rgb`)
- `width` (default: `640`)
- `height` (default: `400`)
- `fps` (default: `30.0`)
- `queue_size` (default: `4`)

Voorbeeld:

```bash
ros2 run my_depthai_python_template depthai_template --ros-args -p topic_name:=camera/rgb_fast -p fps:=20.0
```


### Configuratie neuraal netwerk
Om een eigen netwerk te gebruiken plaats je het blob en json bestand van je netwerk in de `resources` map van de `my_depthai` package en configureert het `yolo_spatial_detector_node.launch.py` in de `launch` map van de `my_depthai` package op de volgende regels:

```python
nnName = LaunchConfiguration('nnName', default = "SimpleFruitsYoloV8.blob")
nnConfig = LaunchConfiguration('nnConfig', default = "SimpleFruitsYoloV8.json")
```


### Verklaring ROS topics (`ros2 topic list`)
Deze topics worden gepubliceerd door de Yolo spatial network detector

- `/clicked_point`: Door RViz aangeklikt 3D-punt.
- `/color/ObjectText`: Tekstlabel(s) van gedetecteerde objecten.
- `/color/ObjectText_array`: Array met meerdere objectlabels.
- `/color/camera_info`: Camera-calibratie en intrinsics van de kleurcamera.
- `/color/detections`: Detectiebeeld met resultaten van objectdetectie.
- `/color/detections/compressed`: Gecomprimeerde transportversie van `/color/detections`.
- `/color/detections/compressedDepth`: Depth-transportversie van `/color/detections`.
- `/color/detections/theora`: Theora-gecodeerde stream van `/color/detections`.
- `/color/detections/zstd`: Zstd-gecomprimeerde stream van `/color/detections`.
- `/color/image_rect`: Gecorrigeerd (rectified) kleurbeeld.
- `/color/image_rect/compressed`: Gecomprimeerde transportversie van `/color/image_rect`.
- `/color/image_rect/compressedDepth`: Depth-transportversie van `/color/image_rect`.
- `/color/image_rect/theora`: Theora-gecodeerde stream van `/color/image_rect`.
- `/color/image_rect/zstd`: Zstd-gecomprimeerde stream van `/color/image_rect`.
- `/color/image_w_bouding_boxes`: Kleurbeeld met ingetekende bounding boxes.
- `/color/yolov4_spatial_detections`: 3D YOLO-detectieresultaten (positie + klasse).
- `/initialpose`: Startpose die meestal vanuit RViz wordt gezet.
- `/joint_states`: Huidige gewrichtstoestanden van het robotmodel.
- `/left/camera_info`: Camera-info van de linker camera.
- `/left/image_rect`: Gecorrigeerd beeld van de linker camera.
- `/left/image_rect/compressed`: Gecomprimeerde versie van linker beeld.
- `/left/image_rect/compressedDepth`: Depth-transportversie van linker beeld.
- `/left/image_rect/theora`: Theora-stream van linker beeld.
- `/left/image_rect/zstd`: Zstd-stream van linker beeld.
- `/move_base_simple/goal`: Doelpose die meestal vanuit RViz wordt gezet.
- `/parameter_events`: ROS2-events bij parameterwijzigingen.
- `/right/camera_info`: Camera-info van de rechter camera.
- `/right/image_rect`: Gecorrigeerd beeld van de rechter camera.
- `/right/image_rect/compressed`: Gecomprimeerde versie van rechter beeld.
- `/right/image_rect/compressedDepth`: Depth-transportversie van rechter beeld.
- `/right/image_rect/theora`: Theora-stream van rechter beeld.
- `/right/image_rect/zstd`: Zstd-stream van rechter beeld.
- `/robot_description`: URDF robotbeschrijving.
- `/stereo/camera_info`: Camera-info van de stereo/depth-pipeline.
- `/stereo/converted_depth`: Geconverteerd dieptebeeld.
- `/stereo/converted_depth/compressed`: Gecomprimeerde versie van geconverteerde diepte.
- `/stereo/converted_depth/compressedDepth`: Depth-transportversie van geconverteerde diepte.
- `/stereo/converted_depth/theora`: Theora-stream van geconverteerde diepte.
- `/stereo/converted_depth/zstd`: Zstd-stream van geconverteerde diepte.
- `/stereo/depth`: Ruw of standaard dieptebeeld uit stereo.
- `/stereo/depth/compressed`: Gecomprimeerde versie van `/stereo/depth`.
- `/stereo/depth/compressedDepth`: Depth-transportversie van `/stereo/depth`.
- `/stereo/depth/theora`: Theora-stream van `/stereo/depth`.
- `/stereo/depth/zstd`: Zstd-stream van `/stereo/depth`.
- `/stereo/points`: Pointcloud op basis van stereodiepte.
- `/tf`: Dynamische transformaties tussen frames.
- `/tf_static`: Statische (niet-veranderende) transformaties.

### /color/yolov4_spatial_detections topic voorbeeld: 
Hier is een voorbeeld van een bericht dat wordt gepubliceerd op het `/color/yolov4_spatial_detections` topic, dat de resultaten van een YOLOv4-ruimtelijke detectie bevat. Dit bericht is in YAML-formaat en toont de gedetecteerde objecten, hun klassen, scores, bounding boxes en 3D-posities ten opzichte van de camera.

```yaml
header:
  stamp:
    sec: 1779299981
    nanosec: 965254354
  frame_id: oak_rgb_camera_optical_frame
detections:
- results:
  - class_id: '1'
    score: 0.9013671875
  bbox:
    center:
      position:
        x: 485.0
        y: 270.5
      theta: 0.0
    size_x: 310.0
    size_y: 489.0
  position:
    x: 0.1642097681760788
    y: 0.04885092377662659
    z: 0.906000018119812
  is_tracking: false
  tracking_id: ''
  ```