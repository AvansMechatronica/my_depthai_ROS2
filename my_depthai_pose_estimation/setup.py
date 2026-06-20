from pathlib import Path
from glob import glob

from setuptools import find_packages, setup

package_name = 'my_depthai_pose_estimation'


def package_files(directory, install_subdir):
    files = []
    for file_path in Path(directory).rglob('*'):
        if file_path.is_file():
            files.append((f'share/{package_name}/{install_subdir}/{file_path.parent.relative_to(directory)}', [str(file_path)]))
    return files

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
        ] + package_files('pose_estimation/hand_pose/depthai_models', 'hand_pose/depthai_models') 
            + package_files('pose_estimation/hand_pose/utils', 'hand_pose/utils')
            + package_files('pose_estimation/animal_pose/depthai_models', 'animal_pose/depthai_models')
            + package_files('pose_estimation/animal_pose/utils', 'animal_pose/utils')
            + package_files('pose_estimation/human_pose/depthai_models', 'human_pose/depthai_models')
            + package_files('pose_estimation/human_pose/utils', 'human_pose/utils'),
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
            'hand_pose_node = pose_estimation.hand_pose.hand_pose_node:main',
            'hand_pose_markers_node = pose_estimation.hand_pose.hand_pose_markers_node:main',
            'animal_pose_node = pose_estimation.animal_pose.animal_pose_node:main',
            'animal_pose_markers_node = pose_estimation.animal_pose.animal_pose_markers_node:main',
            'human_pose_node = pose_estimation.human_pose.human_pose_node:main',
            'human_pose_markers_node = pose_estimation.human_pose.human_pose_markers_node:main',
        ],
    },
)
