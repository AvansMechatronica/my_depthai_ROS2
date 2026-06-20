from glob import glob

from setuptools import find_packages, setup

package_name = 'my_depthai_object_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        (
            'share/' + package_name + '/kalman_tracking/depthai_models',
            [
                'my_depthai_object_tracking/kalman_tracking/depthai_models/yolov6_nano_r2_coco.RVC2.yaml',
                'my_depthai_object_tracking/kalman_tracking/depthai_models/yolov6_nano_r2_coco.RVC4.yaml',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gerard',
    maintainer_email='GerardAnneHarkema@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'kalman_tracking_node = my_depthai_object_tracking.kalman_tracking.kalman_tracking_node:main',
            'kalman_tracking_markers_node = my_depthai_object_tracking.kalman_tracking.kalman_tracking_markers_node:main',
        ],
    },
)
