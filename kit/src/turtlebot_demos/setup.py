from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot_demos'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS2 Kit Learner',
    maintainer_email='learner@ros2kit.local',
    description='Progressive learning demos for TurtleBot AMR',
    license='MIT',
    entry_points={
        'console_scripts': [
            'break_tf = turtlebot_demos.break_tf:main',
            'break_tf.py = turtlebot_demos.break_tf:main',
            'break_odom = turtlebot_demos.break_odom:main',
            'break_odom.py = turtlebot_demos.break_odom:main',
            'break_costmap = turtlebot_demos.break_costmap:main',
            'break_costmap.py = turtlebot_demos.break_costmap:main',
            'goal_sender = turtlebot_demos.goal_sender:main',
            'goal_sender.py = turtlebot_demos.goal_sender:main',
            'odom_drift_visualizer = turtlebot_demos.odom_drift_visualizer:main',
            'odom_drift_visualizer.py = turtlebot_demos.odom_drift_visualizer:main',
        ],
    },
)
