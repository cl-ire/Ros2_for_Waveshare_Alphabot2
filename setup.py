from setuptools import find_packages, setup

package_name = 'Ros2_for_Waveshare_Alphabot2'

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
    maintainer='Claire Schubert',
    maintainer_email='ubuntu@todo.todo',
    description='Camera Subscriber for the topic /image_raw form the v4l2_camera_node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ir_control = Ros2_for_Waveshare_Alphabot2.ir_control:main',
            'joystick = Ros2_for_Waveshare_Alphabot2.joystick:main',
            'motion = Ros2_for_Waveshare_Alphabot2.motion:main',
            'pan_tilt = Ros2_for_Waveshare_Alphabot2.pan_tilt:main',
            'rgb_leds = Ros2_for_Waveshare_Alphabot2.rgb_leds:main',
            'sensors = Ros2_for_Waveshare_Alphabot2.sensors:main',
            'sound = Ros2_for_Waveshare_Alphabot2.sound:main',
            'alphabot_test = scripts.test:main',
        ],
    },
)
