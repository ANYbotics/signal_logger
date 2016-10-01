cmake_minimum_required(VERSION 2.8.3)
project(rqt_signal_logger)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rqt_gui
    rqt_gui_cpp
    signal_logger_msgs
)

find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})

find_package(OpenGL REQUIRED)

catkin_python_setup()

###########
# sources #
###########
set(rqt_signal_logger_SRCS
    src/SignalLoggerPlugin.cpp
)
###########



###########
# headers #
###########
set(rqt_signal_logger_HDRS
    include/rqt_signal_logger/SignalLoggerPlugin.hpp
    include/rqt_signal_logger/LogElement.hpp
)
###########



############
# ui files #
############
set(rqt_signal_logger_UIS
    resource/signal_logger_plugin.ui
)
############



set(rqt_signal_logger_INCLUDE_DIRECTORIES
    include
    ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
    LIBRARIES rqt_signal_logger
    CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp signal_logger_msgs
)


qt4_wrap_cpp(rqt_signal_logger_MOCS ${rqt_signal_logger_HDRS})
qt4_wrap_ui(rqt_signal_logger_UIS_H ${rqt_signal_logger_UIS})


include_directories(
    ${rqt_signal_logger_INCLUDE_DIRECTORIES}
    ${catkin_INCLUDE_DIRS}
)


add_library(rqt_signal_logger
    ${rqt_signal_logger_SRCS}
    ${rqt_signal_logger_MOCS}
    ${rqt_signal_logger_UIS_H}
)

add_dependencies(rqt_signal_logger ${catkin_EXPORTED_TARGETS})

target_link_libraries(rqt_signal_logger
    ${catkin_LIBRARIES}
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
    ${OPENGL_LIBRARIES}
    glut GL GLU GLEW
)


find_package(class_loader)
class_loader_hide_library_symbols(rqt_signal_logger)


install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(TARGETS rqt_signal_logger
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


install(PROGRAMS scripts/rqt_signal_logger
    DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_signal_logger
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)