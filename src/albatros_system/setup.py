from setuptools import find_packages, setup

package_name = 'albatros_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='buse',
    maintainer_email='buse@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'camera_node = albatros_system.camera_node:main',
        'yolo_node = albatros_system.yolo_node:main',
        'control_node = albatros_system.control_node:main',
        'imu_sensor_node = albatros_system.imu_sensor_node:main',
        'gps_sensor_node = albatros_system.gps_sensor_node:main',
        'mesafe_sensor_node = albatros_system.mesafe_sensor_node:main',
        'costmap_node = albatros_system.costmap_node:main',
        'mission_node = albatros_system.mission_node:main',
        'state_node = albatros_system.state_node:main',
        'yolo_mesafe_node = albatros_system.yolo_mesafe_node:main',
        'duba_fusion_node = albatros_system.duba_fusion_node:main',
    ],
    }
)   