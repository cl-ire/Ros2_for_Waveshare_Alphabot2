cmake_minimum_required(VERSION 3.8)
project(ros2_for_waveshare_alphabot2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Alphabot2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IR.msg"
  "msg/Line_Follow.msg"
  "msg/Line_Follow_Stamped.msg"
  "msg/Obstacle_Stamped.msg"
  "msg/Pan_Tilt.msg"
  "msg/Pan_Tilt_Stamped.msg"
  "msg/RGB_LED.msg"
  DEPENDENCIES std_msgs
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
    DESTINATION lib/${PROJECT_NAME}
)