cmake_minimum_required(VERSION 3.5.1)
project(TrinaPointAndClick)

SET(BUILD_SHARED_LIBS ON)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
	message_generation
        )
find_package(OpenCV REQUIRED)
add_message_files(
    FILES
    Marker.msg
    MarkerArray.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
)
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES MultipleMarkerTracker
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
        #  DEPENDS system_lib
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_STANDARD 11)

set(ApplicationName CVgui)

add_executable(${ApplicationName} main.cpp AuxCameraDisplay.cpp AuxCameraDisplay.h)
target_link_libraries( ${ApplicationName} ${OpenCV_LIBS} ${catkin_LIBRARIES})
set_property(TARGET ${ApplicationName} PROPERTY CXX_STANDARD 11)
install(TARGETS ${ApplicationName} DESTINATION bin)
