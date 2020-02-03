cmake_minimum_required(VERSION 2.8.3)
project(march_safety)

add_compile_options(-std=c++14 -Wall -Wextra -Werror)

find_package(catkin REQUIRED COMPONENTS
    march_shared_resources
    roscpp
    sensor_msgs
    sound_play
    std_msgs
    urdf
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    march_shared_resources
    roscpp
    sensor_msgs
    sound_play
    std_msgs
    urdf
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    append_coverage_compiler_flags()
endif()

add_library(${PROJECT_NAME}
    src/input_device_safety.cpp
    src/safety_handler.cpp
    src/temperature_safety.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/safety_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

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

# Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(${PROJECT_NAME}_temperature_test
        test/launch/${PROJECT_NAME}_temperature.test
        test/rostest/temperature_error_test.cpp
        test/rostest/no_temperature_error_test.cpp
        test/rostest/temperature_parameterized_test.cpp
    )
    add_rostest_gtest(${PROJECT_NAME}_connection_lost_test
        test/launch/${PROJECT_NAME}_connection_lost.test
        test/rostest/connection_lost_test.cpp
    )

    add_rostest_gtest(${PROJECT_NAME}_connection_never_started_test
        test/launch/${PROJECT_NAME}_connection_never_started.test
        test/rostest/connection_never_started_test.cpp
    )

    add_rostest_gtest(${PROJECT_NAME}_connection_not_lost_test
        test/launch/${PROJECT_NAME}_connection_not_lost.test
        test/rostest/connection_not_lost_test.cpp
    )
    target_link_libraries(${PROJECT_NAME}_temperature_test ${catkin_LIBRARIES})
    target_link_libraries(${PROJECT_NAME}_connection_lost_test ${catkin_LIBRARIES})
    target_link_libraries(${PROJECT_NAME}_connection_never_started_test ${catkin_LIBRARIES})
    target_link_libraries(${PROJECT_NAME}_connection_not_lost_test ${catkin_LIBRARIES})

    if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
        add_code_coverage(
            NAME coverage_report
            DEPENDENCIES
            ${PROJECT_NAME}_temperature_test
            ${PROJECT_NAME}_connection_lost_test
            ${PROJECT_NAME}_connection_never_started_test
            ${PROJECT_NAME}_connection_not_lost_test
        )
    endif()
endif()
