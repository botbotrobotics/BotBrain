from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bot_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='botbot',
    maintainer_email='atofetti@Botbot.bot',
    description='YOLO lifecycle node for ROS 2 (Ultralytics/TensorRT)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = bot_yolo.yolo_node:main',
        ],
    },
)