cmake_minimum_required(VERSION 3.5.1)
project(signal_logger_python)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS)
catkin_python_setup()

#############
## Install ##
#############

install(PROGRAMS
  bin/signal_logger_reader
  bin/signal_logger_reader.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(cmake_python_coverage QUIET)
  if(cmake_python_coverage_FOUND)
    add_pytest_with_coverage(test/test_signal_logger_python.py signal_logger)
  endif(cmake_python_coverage_FOUND)
endif()