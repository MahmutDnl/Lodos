import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'albatros_simple'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'models'),
            glob(os.path.join('models', '*.hef'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LODOS Takımı',
    maintainer_email='lodos@todo.todo',
    description='LODOS Albatros İDA - Sade ROS2 Python Paket İskeleti',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = albatros_simple.camera_node:main',
            'yolo_node = albatros_simple.yolo_node:main',
            'mission_node = albatros_simple.mission_node:main',
            'parkur_gecis_node = albatros_simple.parkur_gecis_node:main',
        ],
    },
)
