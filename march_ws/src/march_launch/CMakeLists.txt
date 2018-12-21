cmake_minimum_required(VERSION 2.8.3)
project(march_launch)
add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
        march_control
        march_description
        march_gazebo
        march_state_machine
        march_state_machine_service
        rostest
        )

catkin_package()
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    add_rostest_gtest(${PROJECT_NAME}-test launch/simple-test.test
            test/ExampleTest.cpp
            )
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} march_state_machine_service)
endif ()
