cmake_minimum_required(VERSION 2.8.3)
project(march_input_device)

add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
        march_custom_msgs
        march_main
        roscpp
        roslint
        )

catkin_package(
        CATKIN_DEPENDS
)

# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
        "src/*.cpp"
        "src/*.h"
        )

roslint_cpp(${lintfiles})
include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(input_device_node src/input_device_node.cpp)
add_dependencies(input_device_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(input_device_node ${catkin_LIBRARIES})