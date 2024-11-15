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
    maintainer='DanielFLopez1620',
    maintainer_email='dfelipe.lopez@gmail.com',
    description='Package oriented for CV of g_mov_project',
    license='BSD 3-Clause License',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fall_detect_img_raw_gui = g_mov_opencv_py.fall_detect_img_raw_gui:main',
            'fall_detect_img_raw_no_gui = g_mov_opencv_py.fall_detect_img_raw_no_gui:main',
            'get_people_centered = g_mov_opencv_py.get_people_centered:main',
            'get_people_in_frame = g_mov_opencv_py.get_people_in_frame:main',
        ],
    },
)
