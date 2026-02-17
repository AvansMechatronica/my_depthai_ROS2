# my_depthai_ROS2

ROS 2 pakket voor DepthAI OAK-camera’s met voorbeelden voor stereo, objectdetectie, TF-publicatie en bounding box visualisatie. Dit pakket bevat launch-bestanden, voorbeeldscripts en C++ nodes voor het gebruik van DepthAI in ROS 2.

## Inhoud
- [Overzicht](#overzicht)
- [Functies](#functies)
- [Vereisten](#vereisten)
- [Installatie](#installatie)
- [Build & run](#build--run)
- [Launch-bestanden](#launch-bestanden)
- [Scripts](#scripts)
- [Parameters](#parameters)
- [Modellen (resources)](#modellen-resources)
- [RViz-configuraties](#rviz-configuraties)
- [Troubleshooting](#troubleshooting)
- [Documentatie](#documentatie)
- [Licentie](#licentie)

## Overzicht
Dit pakket is bedoeld als template en voorbeeldimplementatie voor het programmeren van DepthAI camera’s in ROS 2. Het biedt:
- Stereo pointcloud publicatie
- Objectdetectie met YOLO (spatiale detecties)
- Publicatie van TF-frames
- Voorbeelden voor het publiceren van bounding boxes

## Functies
- DepthAI stereo pipeline met pointcloud output
- YOLO spatial detector node (C++)
- Extra conversie van DepthAI spatial detections
- Python scripts voor TF en bounding boxes
- Vooraf geconfigureerde RViz-sessies

## Vereisten
- Ubuntu (getest op Linux)
- ROS 2 (Foxy/Humble of compatibel)
- DepthAI SDK en drivers
- colcon

> Tip: Zorg dat je camera goed herkend wordt (USB 3.0 aanbevolen).

## Installatie
1. Clone de repository in je ROS 2 workspace, bijvoorbeeld:
	- `~/my_depthai_ws/src/my_depthai_ROS2`
2. Controleer of alle dependencies geïnstalleerd zijn (DepthAI en ROS 2 packages).

## Build & run
1. Build de workspace:
	- `colcon build --symlink-install`
2. Source de workspace:
	- `source install/setup.bash`

## Launch-bestanden
De launch-bestanden staan in [my_depthai/launch](my_depthai/launch):
- [my_depthai/launch/stereo.launch.py](my_depthai/launch/stereo.launch.py): Stereo pipeline met pointcloud.
- [my_depthai/launch/yolo_spatial_detector_node.launch.py](my_depthai/launch/yolo_spatial_detector_node.launch.py): YOLO spatial detector.
- [my_depthai/launch/publish_tf.launch.py](my_depthai/launch/publish_tf.launch.py): TF publicatie.
- [my_depthai/launch/publish_bouding_boxes.launch.py](my_depthai/launch/publish_bouding_boxes.launch.py): Bounding boxes publicatie.
- [my_depthai/launch/stereo_circle_detector.launch.py](my_depthai/launch/stereo_circle_detector.launch.py): Voorbeeld met cirkeldetectie.
- [my_depthai/launch/my_yolov4_publisher_ex.launch.py](my_depthai/launch/my_yolov4_publisher_ex.launch.py): Voorbeeld YOLOv4 pipeline.

## Scripts
De Python scripts staan in [my_depthai/scripts](my_depthai/scripts):
- [my_depthai/scripts/publisch_tf.py](my_depthai/scripts/publisch_tf.py): Publiceert TF frames.
- [my_depthai/scripts/publisch_bouding_boxes.py](my_depthai/scripts/publisch_bouding_boxes.py): Publiceert bounding boxes.
- [my_depthai/scripts/circle_detector.py](my_depthai/scripts/circle_detector.py): Detectie van cirkels.
- [my_depthai/scripts/workspace_from_markers.py](my_depthai/scripts/workspace_from_markers.py): Hulpfuncties voor markers.

## Parameters
Camera-instellingen staan in [my_depthai/params/camera](my_depthai/params/camera):
- [my_depthai/params/camera/color.yaml](my_depthai/params/camera/color.yaml)
- [my_depthai/params/camera/left.yaml](my_depthai/params/camera/left.yaml)
- [my_depthai/params/camera/right.yaml](my_depthai/params/camera/right.yaml)

## Modellen (resources)
Voorbeeldmodellen en labels staan in [my_depthai/resources](my_depthai/resources):
- YOLOv4, YOLOv5 en YOLOv7 blobs
- Example labelbestanden (zoals simplefruits)

Let op: kies in je launch of code altijd het juiste modelbestand en labelbestand.

## RViz-configuraties
Gebruik de meegeleverde RViz-configs in [my_depthai/rviz](my_depthai/rviz):
- [my_depthai/rviz/stereoPointCloud.rviz](my_depthai/rviz/stereoPointCloud.rviz)
- [my_depthai/rviz/spatialDetections.rviz](my_depthai/rviz/spatialDetections.rviz)
- [my_depthai/rviz/stereoDetectedCirclesPointCloud.rviz](my_depthai/rviz/stereoDetectedCirclesPointCloud.rviz)

## Troubleshooting
- Geen camera gevonden: controleer USB 3.0 en `lsusb`.
- Geen beeld: controleer of de juiste launch en camera parameters geladen zijn.
- Detecties zijn leeg: controleer of het juiste model en labels gekozen zijn.

## Documentatie
Uitgebreide documentatie staat hier:
- [Documentatie](https://avansmechatronica.github.io/my_depthai_ROS2/)

## Licentie
Zie [licence.md](licence.md).