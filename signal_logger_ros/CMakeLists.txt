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
    signal_logger_std
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES signal_logger_ros
 CATKIN_DEPENDS signal_logger roco std_msgs geometry_msgs kindr_msgs signal_logger_msgs signal_logger_std
# DEPENDS system_lib
)

###########
## Build ##
###########
add_definitions(--std=c++11)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


add_library(signal_logger_ros SHARED 
    src/SignalLoggerRos.cpp
)

add_dependencies(signal_logger_ros ${catkin_EXPORTED_TARGETS})
				
target_link_libraries(signal_logger_ros
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
