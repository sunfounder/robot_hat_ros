from setuptools import find_packages, setup

package_name = 'robot_hat_ros'

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
    maintainer='sunfounder',
    maintainer_email='service@sunfounder.com',
    description='Robot Hat package for ROS',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic = robot_hat_ros.ultrasonic:main',
            'mcu = robot_hat_ros.mcu:main',
            'grayscale_3ch = robot_hat_ros.grayscale_3ch:main',
            'test = robot_hat_ros.test:main',
        ],
    },
)
