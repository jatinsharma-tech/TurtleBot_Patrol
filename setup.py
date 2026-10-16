from setuptools import setup
from glob import glob

package_name = 'turtlebot_patrol'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ishita',
    maintainer_email='ishitachotia112@gmail.com',
    description='TurtleBot3 patrol with obstacle avoidance and e-stop',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_obstacle_detector = turtlebot_patrol.lidar_obstacle_detector:main',
            'patrol_controller = turtlebot_patrol.patrol_controller:main',
            'emergency_stop_server = turtlebot_patrol.emergency_stop_server:main',
            'telemetry_logger = turtlebot_patrol.telemetry_logger:main',
        ],
    },
)
