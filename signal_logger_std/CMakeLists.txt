cmake_minimum_required(VERSION 2.8.3)
project(signal_logger_std)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS 
    signal_logger
    roco
)

# Declare this project as a catkin package
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES signal_logger_std
 CATKIN_DEPENDS signal_logger roco
# DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "-fPIC")
add_definitions(--std=c++11)

find_package(Eigen3)
find_package(Kindr)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Kindr_INCLUDE_DIRS})

set(SRCS
    src/LoggerStd.cpp
)

add_library(signal_logger_std SHARED ${SRCS})
target_link_libraries(signal_logger_std
    ${catkin_LIBRARIES}
)
