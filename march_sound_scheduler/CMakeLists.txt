cmake_minimum_required(VERSION 2.8.3)
project(march_sound_scheduler)
add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
    march_shared_resources
    roscpp
    sound_play
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    march_shared_resources
    roscpp
    sound_play
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/scheduled_sound.cpp
    src/scheduler.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/sound_scheduler_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch sound
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
