cmake_minimum_required(VERSION 3.8)
project(ros2_for_waveshare_alphabot2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IR.msg"
  "msg/LineFollow.msg"
  "msg/LineFollowStamped.msg"
  "msg/ObstacleStamped.msg"
  "msg/PanTilt.msg"
  "msg/PanTiltStamped.msg"
  "msg/RGBLED.msg"
  DEPENDENCIES std_msgs
)

install(
  DIRECTORY
  lib/ros2_for_waveshare_alphabot2
  DESTINATION lib
)

install(
    PROGRAMS
    ros2_for_waveshare_alphabot2/ir_control.py
    ros2_for_waveshare_alphabot2/joystick.py
    ros2_for_waveshare_alphabot2/motion.py
    ros2_for_waveshare_alphabot2/pan_tilt.py
    ros2_for_waveshare_alphabot2/rgb_leds.py
    ros2_for_waveshare_alphabot2/sensors.py
    ros2_for_waveshare_alphabot2/sound.py
    ros2_for_waveshare_alphabot2/test.py
    DESTINATION libexec/ros2_for_waveshare_alphabot2
)

ament_package()