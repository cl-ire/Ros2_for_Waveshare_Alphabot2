import os
from glob import glob
from setuptools import setup

from setuptools import find_packages, setup

package_name = 'ros2_for_waveshare_alphabot2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py')))
        ['libexec/ros2_for_waveshare_alphabot2', glob(os.path.join('libexec', '*'))]
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Claire Schubert',
    maintainer_email='ubuntu@todo.todo',
    description='',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ir_control = ros2_for_waveshare_alphabot2.ir_control:main',
            'joystick = ros2_for_waveshare_alphabot2.joystick:main',
            'motion = ros2_for_waveshare_alphabot2.motion:main',
            'pan_tilt = ros2_for_waveshare_alphabot2.pan_tilt:main',
            'rgb_leds = ros2_for_waveshare_alphabot2.rgb_leds:main',
            'sensors = ros2_for_waveshare_alphabot2.sensors:main',
            'sound = ros2_for_waveshare_alphabot2.sound:main',
            'alphabot2_test = ros2_for_waveshare_alphabot2.test:main',
        ],
    },
)
