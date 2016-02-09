cmake_minimum_required(VERSION 2.8.3)
project(signal_logger_ros)

add_definitions(--std=c++11)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

catkin_simple()

cs_add_library(${PROJECT_NAME} SHARED 
    src/LoggerRos.cpp
    src/LogElementBase.cpp
)
add_dependencies(${PROJECT_NAME}
				std_msgs_generate_messages_cpp 
				geometry_msgs_generate_messages_cpp 
				kindr_msgs_generate_messages_cpp 
				signal_logger_msgs_generate_messages_cpp
				)
				
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

cs_install()

cs_export()

