cmake_minimum_required(VERSION 2.8.3)
project(thmos_dynup)
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        bitbots_msgs
        actionlib
        std_msgs
        roslib
)

catkin_package(
        
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

set(SOURCES
        src/dynup_node.cpp)
        
add_executable(DynupNode  ${SOURCES})
target_link_libraries(DynupNode ${catkin_LIBRARIES})
