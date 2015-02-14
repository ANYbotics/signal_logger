cmake_minimum_required(VERSION 2.8.3)
project(signal_logger)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
    kindr
)

find_package(Eigen3 REQUIRED)

# Declare this project as a catkin package
catkin_package(
 INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
 LIBRARIES signal_logger
 CATKIN_DEPENDS kindr
# DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "-fPIC")
add_definitions(--std=c++11)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

set(SRCS
    src/logger.cpp
    src/LoggerBase.cpp
    src/LoggerNone.cpp
)

add_library(signal_logger SHARED ${SRCS})
target_link_libraries(signal_logger
    ${catkin_LIBRARIES}
)
