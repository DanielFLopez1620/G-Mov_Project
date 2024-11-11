from setuptools import find_packages, setup

package_name = 'g_mov_opencv_py'

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
    maintainer='dan1620',
    maintainer_email='dfelipe.lopez@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fall_detect_img_raw_gui = g_mov_opencv_py.fall_detect_img_raw_gui:main',
            'fall_detect_img_raw_no_gui = g_mov_opencv_py.fall_detect_img_raw_no_gui:main',
        ],
    },
)
