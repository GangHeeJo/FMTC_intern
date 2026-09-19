from setuptools import find_packages, setup
import os
from glob import glob

package_name = 's1_stack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 's1_stack', 'launch'), glob('launch/*.py')),
        (os.path.join('share', 's1_stack', 'config'), glob('config/*.yaml')),
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
        'joy_to_twist = s1_stack.joy_to_twist:main',
        'cmd_mux = s1_stack.cmd_mux:main',
        'serial_sender = s1_stack.serial_sender:main',
        'decision_auto = s1_stack.decision_auto:main',
        ],
    },
)
