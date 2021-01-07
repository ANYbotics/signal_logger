cmake_minimum_required(VERSION 3.5.1)
project(signal_logger_doc)

#--------------------------#
# Documentation

add_definitions(-std=c++11)

find_package(Doxygen)

IF(DOXYGEN)
  find_package(catkin REQUIRED COMPONENTS
    signal_logger
    signal_logger_ros
    signal_logger_std
    signal_logger_msgs
    rqt_signal_logger
  )

  catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS
      signal_logger
      signal_logger_ros
      signal_logger_std
      signal_logger_msgs
      rqt_signal_logger
    DEPENDS
  )

  # This is important to resolve the symlink of this doc package and allows to refer to packages in subfolders and READMEs outside of a package.
  get_filename_component(REAL_PACKAGE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/" REALPATH)

  # Set some environment variables
  set(DOXYGEN_SOURCE_DIRS "\"${REAL_PACKAGE_PATH}/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${REAL_PACKAGE_PATH}/../signal_logger/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${REAL_PACKAGE_PATH}/../signal_logger_core/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${REAL_PACKAGE_PATH}/../signal_logger_msgs/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${REAL_PACKAGE_PATH}/../signal_logger_ros/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${REAL_PACKAGE_PATH}/../signal_logger_std/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${REAL_PACKAGE_PATH}/../rqt_signal_logger/\"")
  Set(DOXYGEN_IMAGE_DIRS "\"${REAL_PACKAGE_PATH}/figures/\"" )
  Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  MAKE_DIRECTORY(${HTML_DIR})

  # Add configure file
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.config.in ${DOCSOURCE_DIR}/doxygen.config IMMEDIATE)

  # Build doxygen.
  add_custom_target(
    ${PROJECT_NAME} ALL
    COMMAND ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config
    WORKING_DIRECTORY ${DOCSOURCE_DIR}
    COMMENT "${PROJECT_NAME}: Generating API documentation."
    VERBATIM
  )
  #ADD_CUSTOM_TARGET(doc ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)

  # Unset cache files
  unset(${DOXYGEN_SOURCE_DIRS} CACHE)
  unset(${DOXYGEN_IMAGE_DIRS} CACHE)
  unset(${DOCSOURCE_DIR} CACHE)
  unset(${HTML_DIR} CACHE)
  unset(${REAL_PACKAGE_PATH} CACHE)
ELSE(DOXYGEN)
  message(FATAL_ERROR "Doxygen not found")
ENDIF(DOXYGEN)
