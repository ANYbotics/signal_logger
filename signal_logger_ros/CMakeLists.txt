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

set(EXECUTABLE_OUTPUT_PATH 	${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH 	${PROJECT_SOURCE_DIR}/lib)

# Noisily default to Release build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  	message(STATUS "Setting build type to 'Release' as none was specified.")
  	set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
	"Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()


set(CMAKE_CXX_FLAGS "-fPIC")
add_definitions(--std=c++11)


find_package(Eigen3)
find_package(Kindr)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Kindr_INCLUDE_DIRS})

set(SRCS
    src/LoggerRos.cpp
)

add_library(signal_logger_ros SHARED ${SRCS})
add_dependencies(signal_logger_ros std_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp kindr_msgs_generate_messages_cpp signal_logger_msgs_generate_messages_cpp)
target_link_libraries(signal_logger_ros
    ${catkin_LIBRARIES}
)
