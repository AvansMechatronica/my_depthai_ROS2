from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'my_depthai_python'

data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]


def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_files(data_files, ['launch/', 'rviz/', 'resources/', 'config/']),
    install_requires=[
        'setuptools',
        'depthai>=3.6.0',
    ],
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
            'depthai_template=my_depthai_python.depthai_template:main',
            'pointcloud_from_images=my_depthai_python.tools.pointcloud_from_images:main',
            'spatial_detector=my_depthai_python.spatial_detector:main',
            'publisch_tf=my_depthai_python.tools.publisch_tf:main',
        ],
    },
)
