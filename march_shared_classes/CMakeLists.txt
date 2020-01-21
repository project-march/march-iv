cmake_minimum_required(VERSION 2.8.3)
project(march_shared_classes)

find_package(catkin REQUIRED COMPONENTS
    march_shared_resources
)

catkin_python_setup()
catkin_package()

if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test/limits_test.py)
    catkin_add_nosetests(test/subgait_test.py)
endif()

