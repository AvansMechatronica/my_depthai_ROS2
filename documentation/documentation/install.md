# Installatie van de DepthAI-repository

Hier wordt beschreven hoe je de repository kan verkrijgen, kunt bouwen en vervolgens kunt testen.

## Development computer
Als in dit document gesproken wordt over een development-computer dan wordt hiermee bedoeld de laptop/computer waarop je de software in ROS2 ontwikkelt.

## Voorbereidingen

### Installeer udev rules voor de camera

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Cloning de ROS2 DepthAI template
Voor het maken van de Depthai ROS2 template maak je gebruik van een Github repository. Je kunt er voor kiezen om deze clone onder een eigen account van Github te plaatsen (1e keuze hieronder). Je kunt daarna eenvoudig backup's van je werk maken naar je eigen Github account.

> we maken gebruik van een prefix my_ur in de packages van de repository om onderscheid te maken met de standaard DepthAIs packages.

:::::{card}

::::{tab-set}

:::{tab-item} Zonder GIT-repository support

* Je kunt de workspace als volgt creëren
```bash
mkdir -p ~/my_depthai_ws/src
cd ~/my_depthai_ws/src
git clone https://github.com/AvansMechatronica/my_depthai_ROS2.git
```

:::

:::{tab-item} Met GIT-repository support

* Maak een account aan bij [Github](https://github.com/) en login op dit account

* Open de [my_depthai_ROS2](https://github.com/AvansMechatronica/my_depthai_ROS2) repository

* Maak een Fork van de repository naar je eigen Github account door op het **Fork icoon**  te klikken:

![image](../images/fork.jpg)

* Volg de instructies, maar wijzig de naam van de nieuwe repository niet. Bevestig met **Create Fork**

* Nu kun je de workspace als volgt creëren

```bash
mkdir -p ~/my_depthai_ws/src
cd ~/my_depthai_ws/src
git clone https://github.com/<jouw_account_naam>/my_depthai_ROS2.git
```

*ps. Het gebruik van github (zoals add, commit & push commando's) valt  buiten de scope van deze documentatie*

:::

::::

:::::


## Installatie van DepthAI support packages

Met onderstaand commando worden alle benodigde software voor de template geinstalleerd en de workspace gebouwd met colcon.

```bash
cd ~/my_depthai_ws/
rosdep update
rosdep install --ignore-src --from-paths src -y
```



## Installatie van de DepthAI python dependency

::::::{card}

:::::{tab-set}

::::{tab-item} In system python environment
```bash

pip install depthai --break-system-packages
pip install depthai-nodes --break-system-packages
pip install opencv-python --break-system-packages
```

::::

::::{tab-item} In virtuele environment

```bash
cd ~/my_depthai_ws/src/my_depthai_ROS2
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip

# DepthAI python dependency
python -m pip install depthai
```

:::{note}
Als je al ergens een andere virtual environment hebt, en deze wordt gesourced in een `install/setup.bash` van een andere workspace, dan kun je deze ook gebruiken. Zorg er dan wel voor dat de DepthAI python dependency is geïnstalleerd in diezelfde virtual environment.
:::
::::

:::::

::::::


## Bouwen van de workspace
### Bouw de workspace
```bash
cd ~/my_depthai_ws
colcon build --symlink-install
source install/setup.bash
```

### Voeg environment toe aan bashrc
Om de installatie automatisch te sourcen bij het openen van een nieuwe terminal, kun je de volgende regel toevoegen aan je `~/.bashrc` bestand:
```bash
source ~/my_depthai_ws/install/setup.bash
```
 Je kunt dit met het volgende commando doen:
```bash
echo "source ~/my_depthai_ws/install/setup.bash" >> ~/.bashrc
```


## Testen van de installatie
Je kunt de installatie testen door onderstaand commando. Sluit de DepthAi camera aan op de computer en start de volgende ROS2 applicatie

```bash
ros2 launch my_depthai_python depthai_template.launch.py
```

