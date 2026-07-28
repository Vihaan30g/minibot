import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'minibot_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vihaa',
    maintainer_email='vihaan30g@gmail.com',
    description='Kinematics controller node for Minibot differential drive robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minibot_controller_node = minibot_control.minibot_controller:main'
        ],
    },
)