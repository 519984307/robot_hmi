##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(robot_hmi)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#set(CMAKE_PREFIX_PATH /opt/ros/noetic)	# 这是ubuntu20.04  18： /opt/ros/medolic
#添加c++11的支持
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
##############################################################################
# Catkin
##############################################################################
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS rviz roscpp sensor_msgs
    cv_bridge
    std_msgs
    image_transport
    )

find_package(Qt5 REQUIRED Core Widgets Multimedia Concurrent PrintSupport)
set(QT_LIBRARIES Qt5::Widgets Qt5::Multimedia Qt5::Concurrent Qt5::PrintSupport)
include_directories(    # 包含路径后，可以直接引入头文件名字而不需要路径了
    ${catkin_INCLUDE_DIRS}
    include/robot_hmi
    include/RvizPanel
    include/robot_msg
    include/module
    include/RoadPlot
    include/algorithm
)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
    include/robot_hmi/*.hpp *.h
    include/robot_msg/*.hpp *.h
    include/RoadPlot/*.hpp *.h
    include/RvizPanel/*.hpp *.h
    include/module/*.hpp *.h
)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
    src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(robot_hmi ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(robot_hmi ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS robot_hmi RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

