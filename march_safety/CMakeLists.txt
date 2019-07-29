cmake_minimum_required(VERSION 2.8.3)
project(march_safety)
add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
        code_coverage
        march_shared_resources
        roscpp
        roslint
        rostest
        sensor_msgs
        std_msgs
        urdf
        actionlib
        controller_manager
        xacro
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

set(sourcefiles
        src/SafetyNode.cpp
        include/march_safety/TemperatureSafety.h
        include/march_safety/InputDeviceSafety.h
        include/march_safety/TrajectorySafety.h
        include/march_safety/SafetyHandler.h
        include/march_safety/SafetyType.h
        src/TemperatureSafety.cpp
        src/InputDeviceSafety.cpp
        src/TrajectorySafety.cpp
        src/SafetyHandler.cpp
        )

add_library(march_safety ${sourcefiles})
add_executable(march_safety_node ${sourcefiles})
add_dependencies(march_safety_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(march_safety_node ${catkin_LIBRARIES})

# Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)

    if(ENABLE_COVERAGE_TESTING)
        include(CodeCoverage)
        APPEND_COVERAGE_COMPILER_FLAGS()
    endif()

    add_rostest_gtest(${PROJECT_NAME}-test
            test/march_safety.test
            test/TestTemperatureError.cpp
            test/TestNoTemperatureError.cpp
            test/TestTemperatureParameterized.cpp
            )
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} march_safety gtest gmock)


    add_rostest_gtest(${PROJECT_NAME}-test-connection-lost
            test/march_safety_connection_lost.test
            test/TestConnectionLost.cpp
            )

    add_rostest_gtest(${PROJECT_NAME}-test-connection-never-started
            test/march_safety_connection_never_started.test
            test/TestConnectionNeverStarted.cpp
            )

    add_rostest_gtest(${PROJECT_NAME}-test-connection-not-lost
            test/march_safety_connection_not_lost.test
            test/TestConnectionNotLost.cpp
            )
    target_link_libraries(${PROJECT_NAME}-test-connection-lost ${catkin_LIBRARIES} march_safety gtest gmock)
    target_link_libraries(${PROJECT_NAME}-test-connection-never-started ${catkin_LIBRARIES} march_safety gtest gmock)
    target_link_libraries(${PROJECT_NAME}-test-connection-not-lost ${catkin_LIBRARIES} march_safety gtest gmock)

    add_executable(rrbot test/rrbot.cpp)
    target_link_libraries(rrbot ${catkin_LIBRARIES})

    add_dependencies(tests rrbot)
    add_dependencies(tests ${PROJECT_NAME})

    add_rostest_gtest(${PROJECT_NAME}-trajectory-test
            test/trajectory_controller_safety.test
            test/TestTrajControlPathTolerance.cpp
            )
    target_link_libraries(${PROJECT_NAME}-trajectory-test ${catkin_LIBRARIES} march_safety gtest gmock)

    if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
        add_code_coverage(
                NAME ${PROJECT_NAME}_coverage
                DEPENDS tests
        )
    endif()


endif ()

# Install
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY include/march_safety/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/march_safety/)

# Install library
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

install(FILES ros_control_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})