cmake_minimum_required(VERSION 2.8.3)
project(march_safety)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
    march_shared_resources
    roscpp
    roslint
    sensor_msgs
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
    std_msgs
    urdf
)

# Roslinter
set(ROSLINT_CPP_OPTS "--linelength=120")

file(GLOB_RECURSE lintfiles
    "include/*.h"
    "src/*.cpp"
)

roslint_cpp(${lintfiles})
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

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
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    find_package(code_coverage REQUIRED)
    find_package(rostest REQUIRED)

    if(ENABLE_COVERAGE_TESTING)
        include(CodeCoverage)
        append_coverage_compiler_flags()
    endif()

    add_rostest_gtest(${PROJECT_NAME}-test
        test/${PROJECT_NAME}.test
        test/TestTemperatureError.cpp
        test/TestNoTemperatureError.cpp
        test/TestTemperatureParameterized.cpp
    )
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${PROJECT_NAME} gtest gmock)

    add_rostest_gtest(${PROJECT_NAME}-test-connection-lost
        test/${PROJECT_NAME}_connection_lost.test
        test/TestConnectionLost.cpp
    )

    add_rostest_gtest(${PROJECT_NAME}-test-connection-never-started
        test/${PROJECT_NAME}_connection_never_started.test
        test/TestConnectionNeverStarted.cpp
    )

    add_rostest_gtest(${PROJECT_NAME}-test-connection-not-lost
        test/${PROJECT_NAME}_connection_not_lost.test
        test/TestConnectionNotLost.cpp
    )
    target_link_libraries(${PROJECT_NAME}-test-connection-lost ${catkin_LIBRARIES} ${PROJECT_NAME} gtest gmock)
    target_link_libraries(${PROJECT_NAME}-test-connection-never-started ${catkin_LIBRARIES} ${PROJECT_NAME} gtest gmock)
    target_link_libraries(${PROJECT_NAME}-test-connection-not-lost ${catkin_LIBRARIES} ${PROJECT_NAME} gtest gmock)

    if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
        add_code_coverage(
            NAME ${PROJECT_NAME}_coverage
            DEPENDS tests
        )
    endif()
endif ()
