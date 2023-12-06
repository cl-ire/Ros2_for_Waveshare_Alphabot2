cmake_minimum_required(VERSION 3.5)
project(ros2_for_waveshare_alphabot2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add your messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IR.msg"
  "msg/LineFollow.msg"
  "msg/LineFollowStamped.msg"
  "msg/ObstacleStamped.msg"
  "msg/PanTilt.msg"
  "msg/PanTiltStamped.msg"
  "msg/RGBLED.msg"
)


ament_package()
