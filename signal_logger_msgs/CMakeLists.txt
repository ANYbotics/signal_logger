cmake_minimum_required(VERSION 3.16.3)
project(signal_logger_msgs)

if("$ENV{ROS_VERSION}" STREQUAL "1")

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
   Int16Stamped.msg
   Int8Stamped.msg
   UInt64Stamped.msg
   UInt32Stamped.msg
   UInt16Stamped.msg
   UInt8Stamped.msg
   BoolStamped.msg
   CharStamped.msg
   StringStamped.msg
   UnsignedCharStamped.msg
   TimeStamped.msg
   PairStringInt.msg
   PairStringIntStamped.msg
   MapStringIntStamped.msg
   PairStringDouble.msg
   PairStringDoubleStamped.msg
   MapStringDoubleStamped.msg
   PairIntDouble.msg
   PairIntDoubleStamped.msg
   MapIntDoubleStamped.msg
   LogElement.msg
)

add_service_files(
    FILES
    GetLoggerElement.srv
    GetLoggerConfiguration.srv
    EditLoggerScript.srv
    SetLoggerElement.srv
    SaveLoggerData.srv
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

else() # ROS version 2

set(PACKAGE_DEPENDENCIES
  builtin_interfaces
  geometry_msgs
  std_msgs
  )

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
  
foreach(PKG IN LISTS PACKAGE_DEPENDENCIES)
  find_package(${PKG} REQUIRED)
endforeach()

set(MSG_FILES
  "msg/BoolMultiArray.msg"
  "msg/Float64MultiArrayStamped.msg"
  "msg/Int64MultiArrayStamped.msg"
  "msg/BoolMultiArrayStamped.msg"
  "msg/Int16MultiArrayStamped.msg"
  "msg/Int8MultiArray.msg"
  "msg/Float32MultiArrayStamped.msg"
  "msg/Int32MultiArrayStamped.msg"
  "msg/Int8MultiArrayStamped.msg"
  "msg/Float64Stamped.msg"
  "msg/Float32Stamped.msg"
  "msg/Int64Stamped.msg"
  "msg/Int32Stamped.msg"
  "msg/Int16Stamped.msg"
  "msg/Int8Stamped.msg"
  "msg/UInt64Stamped.msg"
  "msg/UInt32Stamped.msg"
  "msg/UInt16Stamped.msg"
  "msg/UInt8Stamped.msg"
  "msg/BoolStamped.msg"
  "msg/CharStamped.msg"
  "msg/StringStamped.msg"
  "msg/UnsignedCharStamped.msg"
  "msg_ros2/TimeStamped.msg"
  "msg/PairStringInt.msg"
  "msg/PairStringIntStamped.msg"
  "msg/MapStringIntStamped.msg"
  "msg/PairStringDouble.msg"
  "msg/PairStringDoubleStamped.msg"
  "msg/MapStringDoubleStamped.msg"
  "msg/PairIntDouble.msg"
  "msg/PairIntDoubleStamped.msg"
  "msg/MapIntDoubleStamped.msg"
  "msg/LogElement.msg"
  "srv/GetLoggerElement.srv"
  "srv/GetLoggerConfiguration.srv"
  "srv/EditLoggerScript.srv"
  "srv/SetLoggerElement.srv"
  "srv/SaveLoggerData.srv"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
    ${MSG_FILES}
  DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
  )

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()

endif()
