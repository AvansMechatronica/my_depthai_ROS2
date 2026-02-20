# Applicaties


## Cirkel detector
Deze applicatie demonstreet een cirkel detect in een image en publiceert de gedetecteerd circles isn een nieuw image.

```bash
ros2 launch my_depthai stereo_circle_detector.launch.py
```
![image](../images/circle_detector.png)


## Neural network detector
```bash
ros2 launch my_depthai stereo_circle_detector.launch.py
```
![image](../images/camera_opstelling.png)
![image](../images/nn_objects.jpg)


### Configuratie neuraal netwerk
Om een eigen netwerk te gebruiken plaats je het blob en json bestand van je netwerk in de `resources` map van de `my_depthai` package en configureert het `yolo_spatial_detector_node.launch.py` in de `launch` map van de `my_depthai` package op de volgende regels:

```python
nnName = LaunchConfiguration('nnName', default = "SimpleFruitsYoloV8.blob")
nnConfig = LaunchConfiguration('nnConfig', default = "SimpleFruitsYoloV8.json")
```