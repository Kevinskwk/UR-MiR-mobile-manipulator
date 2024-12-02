from setuptools import find_packages, setup

package_name = 'vlm_ros_interface'

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
    maintainer='Kevin Ma',
    maintainer_email='kevinskwk@gmail.com',
    description='VLM ROS2 interface',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_interface = vlm_ros_interface.robot_interface:main'
        ],
    },
)
