cmake_minimum_required(VERSION 2.8.3)
project(signal_logger_std)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS 
    signal_logger
    roco
)

find_package(Eigen3 REQUIRED)

# Declare this project as a catkin package
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES signal_logger_std
 CATKIN_DEPENDS signal_logger roco
# DEPENDS system_lib
)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

set(CMAKE_CXX_FLAGS "-fPIC")
add_definitions(--std=c++11)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${kindr_INCLUDE_DIRS})

add_library(signal_logger_std SHARED 
    src/LoggerStd.cpp
)
target_link_libraries(signal_logger_std
    ${catkin_LIBRARIES}
)
