import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pilbot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tahir',
    maintainer_email='tahirifdn@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pilbot_movement = pilbot_controller.pilbot_movements:main",
            "real_pose_broadcaster = pilbot_controller.real_pose_broadcaster:main",
            "simple_controller = pilbot_controller.simple_controller:main",
            "noisy_controller = pilbot_controller.noisy_controller:main"
        ],
    },
)
