cmake_minimum_required(VERSION 2.8.3)
project(march_launch)
add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
        march_description
        march_simulation
        march_state_machine
        march_state_machine_service
        roslint
        rosserial_python
        rostest
        )

catkin_package()

catkin_python_setup()

install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY resource
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(PROGRAMS scripts/rqt_software_check
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

file(GLOB_RECURSE lintfiles
        "src/*.py"
        )

roslint_python(${lintfiles})

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    add_rostest_gtest(${PROJECT_NAME}-test test/simple-test.test
            test/ExampleTest.cpp
            )
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif ()
