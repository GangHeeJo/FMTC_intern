from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drive_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'drive_teleop', 'launch'), glob('launch/*.py')),
        (os.path.join('share', 'drive_teleop', 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fmtc',
    maintainer_email='fmtc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'joy_to_twist = drive_teleop.joy_to_twist:main',
        'cmd_mux = drive_teleop.cmd_mux:main',
        'serial_sender = drive_teleop.serial_sender:main',
        ],
    },
)
