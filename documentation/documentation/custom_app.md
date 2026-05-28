# Aangepaste applicaties

Je kunt ook een eigen aangepaste applicatie maken voor de Luxonis DepthAI hardware. Dit kan handig zijn als je specifieke functionaliteiten wilt implementeren die niet worden gedekt door de standaardapplicaties in de `my_depthai_ROS2` repository.

## Stappen om een aangepaste applicatie te maken

* Gebruik onderstaande voorbeelden als basis voor jouw aangepaste applicatie, of maak een geheel nieuwe applicatie.

* Kies een voorbeeldprogramma van Luxonis, welke het beste past bij jou vraagstuk
  * [Depthai Examples](https://docs.luxonis.com/software-v3/depthai/examples/)
  * [Oak Examples](https://docs.luxonis.com/software-v3/ai-inference/inference/oak-examples)
  * [GitHub Oak Examples](https://github.com/luxonis/oak-examples)
* Maak van dit voorbeeld programma een ROS node en test of je de ROS-node kunt starten
* Voeg aan de ROS-node topics toe die het mogelijk maken om informatie uit jou beelden met andere nodes te delen.
* Maak eventueel een custom ROS2 message aan.

:::{note}
Bestudeer de Luxonis documentatie goed, zodat je weet welke functionaliteiten er allemaal mogelijk zijn. Er zijn veel voorbeelden beschikbaar die je kunnen helpen bij het maken van jouw aangepaste applicatie. Zie [DepthAI Software](https://docs.luxonis.com/software-v3/) voor meer informatie.
:::


## Python template (DepthAI)
Deze applicatie publiceert RGB-beelden van een OAK-camera op een ROS2 topic. Deze template kan worden gebruikt als basis voor het maken van een aangepaste applicatie die gebruikmaakt van de DepthAI hardware.

```bash
ros2 launch my_depthai_python depthai_template.launch.py
```


### Optionele parameters
De parametsers van deze applicatie kunnen worden aangepast in het `depthai_template.yaml` bestand in de `config` map van de `my_depthai_python` package. De volgende parameters kunnen worden aangepast:

    topic_name: camera/rgb
    camera_info_topic: camera/camera_info
    width: 640
    height: 400
    fps: 30.0
    queue_size: 4

