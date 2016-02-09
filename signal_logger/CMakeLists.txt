cmake_minimum_required(VERSION 2.8.3)
project(signal_logger)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-fPIC")
add_definitions(--std=c++11)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)


# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${kindr_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

set(SRCS
    src/logger.cpp
    src/LoggerBase.cpp
    src/LoggerNone.cpp
)

catkin_simple()

cs_add_library(${PROJECT_NAME} SHARED ${SRCS})
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

cs_install()

cs_export()

