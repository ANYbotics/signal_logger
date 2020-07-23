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
