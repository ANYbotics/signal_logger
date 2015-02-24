cmake_minimum_required(VERSION 2.8.3)
project(signal_logger_ros)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS 
    signal_logger
    roco
    std_msgs
    geometry_msgs
    kindr_msgs
    signal_logger_msgs
)

# Declare this project as a catkin package
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES signal_logger_ros
 CATKIN_DEPENDS signal_logger roco std_msgs geometry_msgs kindr_msgs signal_logger_msgs
# DEPENDS system_lib
)

add_definitions(--std=c++11)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


add_library(signal_logger_ros SHARED 
    src/LoggerRos.cpp
    src/LogElementBase.cpp
)
add_dependencies(signal_logger_ros 
				std_msgs_generate_messages_cpp 
				geometry_msgs_generate_messages_cpp 
				kindr_msgs_generate_messages_cpp 
				signal_logger_msgs_generate_messages_cpp
				)
				
target_link_libraries(signal_logger_ros
    ${catkin_LIBRARIES}
)
