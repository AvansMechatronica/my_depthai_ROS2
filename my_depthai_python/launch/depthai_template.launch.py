from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Deze launchfile start een minimale RGB-stream pipeline:
    # 1) de DepthAI publisher-node,
    # 2) optioneel RViz2 met een standaard visualisatieconfig.
    #
    # Door alle relevante instellingen als launch-argument aan te bieden,
    # kan dezelfde file gebruikt worden voor snelle tests, demos en tuning
    # zonder dat Python-code aangepast hoeft te worden.

    # -----------------------------
    # Camera/publicatie-instellingen
    # -----------------------------
    # topic_name bepaalt op welk ROS-topic de RGB-beelden gepubliceerd worden.
    # Dit maakt integratie met andere nodes eenvoudig (bijv. image_tools,
    # detectie- of loggingnodes) door alleen de launchparameter te wijzigen.
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='camera/rgb',
        description='Output image topic for DepthAI RGB frames',
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='camera/camera_info',
        description='Output CameraInfo topic for DepthAI RGB frames',
    )
    # width/height sturen direct de outputresolutie van de hoststream.
    # Kleinere waardes verlagen CPU/bandbreedte, grotere waardes verhogen detail.
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Output image width in pixels',
    )
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='400',
        description='Output image height in pixels',
    )
    # fps bepaalt hoe vaak frames opgehaald/gepubliceerd worden.
    # Hogere fps geeft vloeiender beeld maar verhoogt load op host en device.
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30.0',
        description='Camera FPS',
    )
    # queue_size bepaalt hoeveel frames in de DepthAI outputqueue mogen staan.
    # Een kleine queue houdt latency laag; een grotere queue vangt korte dips op.
    queue_size_arg = DeclareLaunchArgument(
        'queue_size',
        default_value='4',
        description='DepthAI output queue size',
    )
    # params_file maakt het mogelijk om defaults centraal in YAML te beheren.
    # De launch-argumenten hieronder blijven bruikbaar als runtime-override.
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('my_depthai_python'), 'config', 'depthai_template.yaml']
        ),
        description='Path to YAML file with depthai_template ROS parameters',
    )

    # -----------------------------
    # Visualisatie-instellingen
    # -----------------------------
    # start_rviz laat headless gebruik toe (bijv. via SSH of in CI) door
    # RViz conditioneel te starten.
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Start RViz2 with the configured RViz file',
    )
    # rviz_config wijst standaard naar de package-share configuratie,
    # maar kan via CLI vervangen worden door een custom RViz-profiel.
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('my_depthai_python'), 'rviz', 'depthai_template.rviz']
        ),
        description='Absolute path to the RViz2 configuration file',
    )

    # Hoofdnode van deze launchfile.
    # LaunchConfiguration-waarden worden als ROS-parameters doorgegeven,
    # zodat runtime-instellingen centraal via launch beheerd blijven.
    depthai_node = Node(
        package='my_depthai_python',
        executable='depthai_template',
        name='depthai_publisher',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'topic_name': LaunchConfiguration('topic_name'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'queue_size': LaunchConfiguration('queue_size'),
            }
        ],
    )

    # RViz wordt alleen gestart wanneer start_rviz=true.
    # Dit voorkomt onnodige GUI-processen op systemen zonder display.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    # Volgorde in LaunchDescription:
    # 1) eerst argumentdeclaraties (zodat overrides bekend zijn),
    # 2) dan de functionele node,
    # 3) en tenslotte de optionele visualisatie.
    #
    # Deze structuur maakt de launchfile leesbaar en voorspelbaar.
    return LaunchDescription(
        [
            topic_name_arg,
            camera_info_topic_arg,
            width_arg,
            height_arg,
            fps_arg,
            queue_size_arg,
            params_file_arg,
            start_rviz_arg,
            rviz_config_arg,
            depthai_node,
            rviz_node,
        ]
    )
