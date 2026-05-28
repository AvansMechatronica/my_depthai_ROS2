# AI Netwerk


## Training



### Yolo V8
Er is een Colab notebook trainingesmodel beschikbaar.

[Yolo V8 training](https://colab.research.google.com/drive/1M_Fnquk3dmfuBLfpjgMGVwj35Em1-wfP)

Volg de instructies van het Colab notebook.

### PyTorch
Je kunt ook een eigen script maken met PyTorch, zie: [ultralytics](https://docs.ultralytics.com/models/yolov8/) 

## Conversie van Yolo bestanden naar blob bestanden


Na training dient het netwerkbestand `best.pt` geconverteerd te worden naar een tweetal DepthAI compatible bestanden:

Gebruik hiervoor de [luxonis conversie tool](https://tools.luxonis.com/)

![image](../images/luxonis-tools.png)

Na conversie worden 2 bestanden gegenereerd:
* `*.blob`: Eigenlijke AI Netwerk, welke in de camera wordt geladen

* `*.json` : bestand met karakteristieke eigenschappen van het `blob` bestand zoals labels van de te detecteren objecten.

Je kunt de namen van de bestanden wijzigen naar nieuwe namen zoals b.v. `<my_yolo_network>.blob` en `<my_yolo_network>.json`

Plaats beide bestanden in de `resources` map van de `my_depthai_python` package.

## Uitrollen

Modificeer het `spatial_detector.yaml` uit de `config` map van de `my_depthai_python` package.

Vervang de volgende regels:
```yaml
    blob_name: SimpleFruitsYoloV8.blob
    config_name: SimpleFruitsYoloV8.json
```

door:
```yaml
    blob_name: "<my_yolo_network>.blob"
    config_name: "<my_yolo_network>.json"
```

## Netwerk testen
Sluit de DepthAi camera aan op de computer en start de volgende ROS2 applicatie

```bash
ros2 launch my_depthai_python spatial_detector.launch.py
```