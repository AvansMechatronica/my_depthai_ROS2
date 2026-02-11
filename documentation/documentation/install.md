# Installatie van de UR-template

Hier wordt beschreven hoe je de template kan verkrijgen, kunt bouwen en tenslotte kunt testen.

## Development computer
Als in dit document gesproken wordt over een development-computer dan wordt hiermee bedoeld de laptop/computer waarop je de software in ROS2 ontwikkelt.

## Cloning de ROS2 Universal Robots template
Voor het maken van de ROS2 Universal Robots template maak je gebruik van een Github repository. Je kunt er voor kiezen om deze clone onder een eigen account van Github te plaatsen (1e keuze hieronder). Je kunt daarna eenvoudig backup's van je werk maken naar je eigen Github account.

> we maken gebruik van een prefix my_ur in de packages van de repository om onderscheid te maken met de standaard Universal Robots packages.

:::::{card} 

::::{tab-set}

:::{tab-item} Met GIT-repository support

* Maak een account aan bij [Github](https://github.com/) en login op dit account

* Open de [my_ur_ROS2](https://github.com/AvansMechatronica/my_ur_ROS2) repository

* Maak een Fork van de repository naar je eigen Github account door op het **Fork icoon**  te klikken:

![image](../images/fork.jpg)

* Volg de instructies, maar wijzig de naam van de nieuwe repository niet. Bevestig met **Create Fork**  

* Nu kun je de workspace als volgt creëren

```bash
mkdir -p ~/my_ur_ws/src
cd ~/my_ur_ws/src
git clone https://github.com/<jouw_account_naam>/my_ur_ROS2.git
```

*ps. Het gebruik van github (zoals add, commit & push commando's) valt  buiten de scope van deze documentatie*

:::

:::{tab-item} Zonder GIT-repository support

* Je kunt de workspace als volgt creëren
```bash
mkdir -p ~/my_ur_ws/src
cd ~/my_ur_ws/src
git clone https://github.com/AvansMechatronica/my_ur_ROS2.git
```

:::

::::

:::::



## Installatie van Universal Robot support packages
Met onderstaand commando worden alle benodigde software voor de template geinstalleerd en de workspace gebouwd met colcon.

```bash
cd ~/my_ur_ws/src/my_ur_ROS2/install
./install
```

## Bouwen van de workspace
> Dit is al gebeurd in de installatie. Wijzig je iets in de workspace dan kun je als volgt bouwen.
```bash
# Build the workspace
cd ~/my_ur_ws
colcon build --symlink-install
source install/setup.bash
```

Heb je slechts 1 package gewijzigd dan kun je onderstaand commando gebruiken om betreffende package te bouwen.

```bash
# Build one of the packages in the workspace
cd ~/my_ur_ws
colcon build --symlink-install --packages-select <package_name>
source install/setup.bash
```

## Testen van de installatie
Je kunt de installatie testen door onderstaand commando. Je hebt hiervoor geen fysieke robot of simulatie omgeving nodig.

```bash
ros2 launch my_ur_moveit_config demo.launch.py
```
Je kunt nu in RVIZ het model van de robot-applicatie zien en met de knop `Goal State` een positie kiezen en de weg naar de positie volgen met de `Plan` knop. Stel eventueel eerst een `Start State` in.

>Omdat er geen fysieke robot of Gazebo simulatie is kun je geen `Execute` functies uitvoeren.
