cmake_minimum_required(VERSION 2.8.3)
project(signal_logger_sl)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS 
    signal_logger
    roco
    kindr
)

find_package(Eigen3 REQUIRED)

# Declare this project as a catkin package
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES signal_logger_sl
 CATKIN_DEPENDS signal_logger roco
# DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
add_definitions(--std=c++11)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})


add_library(signal_logger_sl  
 src/LoggerSl.cpp
)
target_link_libraries(signal_logger_sl
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
