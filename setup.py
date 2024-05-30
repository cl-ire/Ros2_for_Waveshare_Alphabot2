from setuptools import find_packages, setup

package_name = 'ros2_for_waveshare_alphabot2'

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
    description='Ros2 node for for the Waveshare Alphabot 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    package_data={
        package_name: [
            'msg/*.msg',
            'launch/*.py',
        ],
    },

    entry_points={
        'console_scripts': [
            'joystick = ros2_for_waveshare_alphabot2.joystick:main',
            'motion = ros2_for_waveshare_alphabot2.motion2:main',
            'pan_tilt = ros2_for_waveshare_alphabot2.pan_tilt:main',
        ],
    },
    
    include_package_data=True,
)