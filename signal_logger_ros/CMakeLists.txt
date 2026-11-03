cmake_minimum_required(VERSION 3.28)
project(signal_logger_ros)
###############
## Configure ##
###############

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(
  -Wall
  -Wextra
  -Wpedantic
)

set(
  PACKAGE_DEPENDENCIES
  std_srvs
  std_msgs
  kindr_msgs
  signal_logger_msgs
  signal_logger_std
  rosbag2_cpp
  message_logger
  geometry_msgs
  signal_logger_core
  rclcpp
)

find_package(ament_cmake REQUIRED)
foreach(PKG ${PACKAGE_DEPENDENCIES})
  find_package(${PKG} REQUIRED)
endforeach()

###########
## Build ##
###########

add_library(${PROJECT_NAME} SHARED src/SignalLoggerRos2.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})

#############
## Install ##
#############
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY
    DESTINATION lib
  ARCHIVE
    DESTINATION lib
  RUNTIME
    DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_${PROJECT_NAME}
    test/test_${PROJECT_NAME}.cpp
    test/test_${PROJECT_NAME}_buffer_size.cpp
    test/test_main.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})

endif()

ament_package()
