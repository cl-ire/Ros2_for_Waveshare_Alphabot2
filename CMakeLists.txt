cmake_minimum_required(VERSION 3.1)
project(Ros2_for_Waveshare_Alphabot2)


################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  DIRECTORY msg
  FILES
  IR.msg
  Line_Follow.msg
  Line_Follow_Stamped.msg
  Obstacle_Stamped.msg
  Pan_Tilt.msg
  Pan_Tilt_Stamped.msg
  RGB_LED.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(

)
