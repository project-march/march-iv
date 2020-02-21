cmake_minimum_required(VERSION 2.8.3)
project(march_gait_scheduler)

add_compile_options(-std=c++14 -Wall -Wextra -Werror)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    control_msgs
    dynamic_reconfigure
    march_shared_resources
    roscpp
    trajectory_msgs
)

generate_dynamic_reconfigure_options(
    config/Scheduler.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    control_msgs
    dynamic_reconfigure
    march_shared_resources
    roscpp
    trajectory_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/scheduler.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/gait_scheduler_node.cpp)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    find_package(code_coverage REQUIRED)
    find_package(rostest REQUIRED)

    if(ENABLE_COVERAGE_TESTING)
        include(CodeCoverage)
        append_coverage_compiler_flags()
    endif()

    add_rostest_gmock(${PROJECT_NAME}-test
        test/main-test.test
        test/TestRunner.cpp
        test/ScheduleOneGaitTest.cpp
        test/ScheduleMultipleGaitsTest.cpp
        test/EndTimeTest.cpp
        test/AcceptanceTest.cpp
        test/CallbackCounter.cpp
    )

    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${PROJECT_NAME})

    if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
        add_code_coverage(
            NAME ${PROJECT_NAME}_coverage
            DEPENDS tests
        )
    endif()
endif ()
