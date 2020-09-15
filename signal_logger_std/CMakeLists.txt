cmake_minimum_required(VERSION 3.5.1)
project(signal_logger_std)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
  message_logger
  signal_logger_core
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS  message_logger signal_logger_core
)

###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
add_definitions(-std=c++11 -Wall)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
    src/SignalLoggerStd.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY matlab
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
