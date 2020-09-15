cmake_minimum_required(VERSION 3.5.1)
project(signal_logger_ros)

# List of necessary dependencies
set(SILOROS_CATKIN_PACKAGES
  message_logger
  signal_logger_core
  signal_logger_std
  signal_logger_msgs
  std_msgs
  std_srvs
  geometry_msgs
  rosbag
)

# Append optional dependencies (signal_logger defines SILO_USE_KINDR)
find_package(catkin REQUIRED COMPONENTS signal_logger_core)
if(SILO_USE_KINDR)
  list(APPEND SILOROS_CATKIN_PACKAGES kindr_msgs)
endif(SILO_USE_KINDR)

# Find depencendies
find_package(catkin REQUIRED COMPONENTS
  ${SILOROS_CATKIN_PACKAGES}
)

# Configure catkin package
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS ${SILOROS_CATKIN_PACKAGES}
)

###########
## Build ##
###########
add_definitions(-std=c++11 -Wall)
include_directories( include )
include_directories( ${catkin_INCLUDE_DIRS} )

add_library( ${PROJECT_NAME} SHARED
  src/SignalLoggerRos.cpp
)

target_link_libraries( ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies( ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
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

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_${PROJECT_NAME}.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries( test_${PROJECT_NAME}
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
      gtest_main
    )
    find_package(cmake_code_coverage QUIET)
    if(cmake_code_coverage_FOUND)
      add_gtest_coverage()
    endif(cmake_code_coverage_FOUND)
  endif()
endif()
