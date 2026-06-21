# Hulpprogramma's en tools

## Hulpprogramma's bij de spatial_detector node
In de `my_depthai_python` package zijn er een aantal hulpprogramma's te vinden die kunnen worden gebruikt in combinatie met de `spatial_detector` node. 

### Transferframes publiceren
Je kunt de TF frames van de camera en gedetecteerde objecten publiceren met het volgende commando:
```bash
ros2 launch my_depthai_python publish_tf.launch.py
```
Na het uitvoeren van dit commando, kun je de TF frames visualiseren in RViz2. Open een nieuwe terminal en start RViz2 met het volgende commando:
```bash
ros2 topic echo /tf
```
In RViz2, voeg een TF display toe en stel de Fixed Frame in op `oak_rgb_camera_optical_frame`. Je zou nu de TF frames van de camera en de gedetecteerde objecten moeten kunnen zien. De gedetecteerde objecten worden weergegeven als frames die zich in de ruimte bevinden ten opzichte van de camera, gebaseerd op de diepte-informatie die door de `spatial_detector` node wordt gepubliceerd. Je kunt de TF frames gebruiken om de positie en oriëntatie van de gedetecteerde objecten te begrijpen in relatie tot de camera, wat nuttig kan zijn voor toepassingen zoals robotmanipulatie, navigatie of augmented reality.

:::{note}
Als je een eigen netwerk gebruikt dien je het publis_tf.yaml bestand in de `config` map van de `my_depthai_python` package aan te passen. Vervang de volgende regel:
```yaml
    nn_archive: SimpleFruitsYoloV8.rvc2.tar.xz
```

door:
```yaml
    nn_archive: "<my_yolo_network>.rvc2.tar.xz"
```


:::

## Pointclouds publiceren
Je kunt pointclouds(diepte afbeeldingen) publiceren met het volgende commando:
```bash
ros2 launch my_depthai_python pointcloud_from_images.launch.py
```
Na het uitvoeren van dit commando, kun je de pointclouds visualiseren in RViz2. Open een nieuwe terminal en start RViz2 met het volgende commando:

```bash
ros2 topic echo stereo/pointcloud
```

In RViz2, voeg een PointCloud2 display toe en stel het Topic in op `/stereo/pointcloud`. Je zou nu de pointclouds moeten kunnen zien die worden gegenereerd op basis van de diepte-informatie van de camera. Deze pointclouds geven een 3D-weergave van de omgeving weer, waarbij elk punt in de pointcloud een specifieke locatie in de ruimte vertegenwoordigt, gebaseerd op de dieptegegevens van de camera. Je kunt deze pointclouds gebruiken voor toepassingen zoals 3D-mapping, objectherkenning of robotnavigatie.