cmake_minimum_required(VERSION 2.8.3)
project(signal_logger)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
)

find_package(Eigen3 REQUIRED)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
 LIBRARIES signal_logger
# CATKIN_DEPENDS
 DEPENDS kindr
)


###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
add_definitions(--std=c++11)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${kindr_INCLUDE_DIRS})

set(SRCS
    src/logger.cpp
    src/LoggerBase.cpp
    src/LoggerNone.cpp
)

add_library(signal_logger SHARED ${SRCS})
target_link_libraries(signal_logger
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
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
