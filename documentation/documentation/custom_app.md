# Aangepaste applicaties

Je kunt ook een eigen aangepaste applicatie maken voor de Luxonis DepthAI hardware. Dit kan handig zijn als je specifieke functionaliteiten wilt implementeren die niet worden gedekt door de standaardapplicaties in de `my_depthai_ROS2` repository.

## Stappen om een aangepaste applicatie te maken

* Maak een ROS package in je <workspace>/src
  * AMENT_PYTON

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

