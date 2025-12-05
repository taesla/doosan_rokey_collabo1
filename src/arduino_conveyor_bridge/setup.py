from setuptools import find_packages, setup

package_name = 'arduino_conveyor_bridge'

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
    maintainer='ilhoon',
    maintainer_email='bae1hon@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_to_topic = arduino_conveyor_bridge.serial_to_topic:main',
            'conveyor_debug_listener = arduino_conveyor_bridge.conveyor_debug_listener:main',
            'robot_conveyor_demo = arduino_conveyor_bridge.robot_conveyor_demo:main',
        ],
    },
)
