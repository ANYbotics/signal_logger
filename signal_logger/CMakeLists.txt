cmake_minimum_required(VERSION 3.16.3)
project(signal_logger)

if("$ENV{ROS_VERSION}" STREQUAL "1")
  # List of necessary dependencies
  set(
    SILO_CATKIN_PACKAGES
    signal_logger_core
    signal_logger_ros
    signal_logger_std
  )

  find_package(catkin REQUIRED COMPONENTS ${SILO_CATKIN_PACKAGES})

  # Configure catkin package
  catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS ${SILO_CATKIN_PACKAGES}
  )

  ###########
  ## Build ##
  ###########
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  set(CMAKE_CXX_STANDARD 17)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
  include_directories(include)
  include_directories(${catkin_INCLUDE_DIRS})

  add_library(${PROJECT_NAME} SHARED src/signal_logger.cpp)

  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

  #############
  ## Install ##
  #############
  install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} PATTERN ".svn" EXCLUDE)
  install(
    TARGETS
      ${PROJECT_NAME}
    ARCHIVE
      DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY
      DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
      target_link_libraries(
        test_${PROJECT_NAME}
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
else() # ROS version 2
  ###############
  ## Configure ##
  ###############

  set(
    PACKAGE_DEPENDENCIES
    signal_logger_core
    signal_logger_ros
    signal_logger_std
  )

  find_package(ament_cmake REQUIRED)
  foreach(PKG ${PACKAGE_DEPENDENCIES})
    find_package(${PKG} REQUIRED)
  endforeach()

  ###########
  ## Build ##
  ###########
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  set(CMAKE_CXX_STANDARD 17)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )

  add_library(${PROJECT_NAME} SHARED src/signal_logger.cpp)
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
    ament_add_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.cpp)
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})

    find_package(cmake_code_coverage QUIET)
    if(cmake_code_coverage_FOUND)
      add_gtest_coverage()
    endif(cmake_code_coverage_FOUND)
  endif()

  ament_package()
endif()
