cmake_minimum_required(VERSION 2.8.3)
project(signal_logger_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   BoolMultiArray.msg
   Float64MultiArrayStamped.msg
   Int64MultiArrayStamped.msg
   BoolMultiArrayStamped.msg
   Int16MultiArrayStamped.msg
   Int8MultiArray.msg
   Float32MultiArrayStamped.msg
   Int32MultiArrayStamped.msg
   Int8MultiArrayStamped.msg
   Float64Stamped.msg
   Float32Stamped.msg
   Int64Stamped.msg
   Int32Stamped.msg
   Int8Stamped.msg
   BoolStamped.msg
   CharStamped.msg
   UnsignedCharStamped.msg
   TimeStamped.msg
   LogElement.msg
)

add_service_files(
    FILES
    GetLoggerElement.srv
    GetLoggerConfiguration.srv
    LoadLoggerScript.srv
    SetLoggerElement.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
)