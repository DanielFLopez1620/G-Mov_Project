from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'g_mov_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DanielFLopez1620',
    maintainer_email='dfelipe.lopez@gmail.com',
    description='Set up of sensors and communication for g_mov_project',
    license='BSD 3-Clause License',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_subscriber = g_mov_base.servo_subscriber:main',
            'accel_publisher = g_mov_base.accel_publisher:main',
            'thingspeak_accel_sub = g_mov_base.thingspeak_accel_sub:main',
            'thingspeak_servo_sub = g_mov_base.thingspeak_servo_sub:main',
            'thingspeak_cam_sub = g_mov_base.thingspeak_cam_sub:main',
            'head_track_with_servo = g_mov_base.head_track_with_servo:main',
            'mosquitto_accel_sub = g_mov_base.mosquitto_accel_sub:main',
            'mosquitto_servo_sub = g_mov_base.mosquitto_servo_sub:main',
            'mosquitto_fall_sub = g_mov_base.mosquitto_fall_sub:main',
            'mosquitto_servo_sub_mod = g_mov_base.mosquitto_servo_sub_mod:main',
            'thingsboard_accel_sub = g_mov_base.thingsboard_accel_sub:main',
            'thingsboard_servo_sub = g_mov_base.thingsboard_servo_sub:main',
            'thingsboard_fall_sub = g_mov_base.thingsboard_fall_sub:main',
            'thingsboard_servo_pub = g_mov_base.thingsboard_servo_pub:main',
        ],
    },
)
