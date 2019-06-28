cmake_minimum_required(VERSION 2.8.3)
project(march_sound_scheduler)
add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
        code_coverage
        march_shared_resources
        roscpp
        roslint
        rostest
        sound_play
        )

catkin_package(
        INCLUDE_DIRS include
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
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(march_sound_scheduler
        src/SoundSchedulerNode.cpp
        src/ScheduledSound.cpp
        include/march_sound_scheduler/Scheduler.h
        include/march_sound_scheduler/ScheduledSound.h
        src/Scheduler.cpp
)

target_link_libraries(march_sound_scheduler ${catkin_LIBRARIES})

add_executable(sound_scheduler_node src/SoundSchedulerNode.cpp)

target_link_libraries(sound_scheduler_node march_sound_scheduler)
add_dependencies(sound_scheduler_node ${catkin_EXPORTED_TARGETS})