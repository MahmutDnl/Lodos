from setuptools import find_packages, setup

package_name = 'albatros_tahta'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LODOS Takımı',
    maintainer_email='lodos@todo.todo',
    description='LODOS Albatros İDA — Tahta Mimarisi Node Paketi',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'kamera_node = albatros_tahta.kamera_node:main',
        'yolo_node = albatros_tahta.yolo_node:main',
	'yolo_mesafe_node = albatros_tahta_yolo_node:main',
        'mesafe_sensor_node = albatros_tahta.mesafe_sensor_node:main',
        'costmap_node = albatros_tahta.costmap_node:main',
        'duba_fusion_node = albatros_tahta.duba_fusion_node:main',
        'imu_node = albatros_tahta.imu_node:main',
        'gps_node = albatros_tahta.gps_node:main',
        'kontrol_node = albatros_tahta.kontrol_node:main',
        'mission_node = albatros_tahta.mission_node:main',
        'state_node = albatros_tahta.state_node:main',
        'karar_node = albatros_tahta.karar_node:main',
    ],
    }
)
