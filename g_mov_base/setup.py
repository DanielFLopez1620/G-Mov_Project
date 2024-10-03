from setuptools import find_packages, setup

package_name = 'g_mov_base'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_subscriber = g_mov_base.servo_subscriber:main',
            'accel_publisher = g_mov_base.accel_publisher:main',
            'thingspeak_accel_sub = g_mov_base.thingspeak_accel_sub:main',
            'thingspeak_servo_sub = g_mov_base.thingspeak_servo_sub:main',
        ],
    },
)
