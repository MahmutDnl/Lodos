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
            'mavros_node = albatros_system.mavros_node:main',
        ],
    },
)
