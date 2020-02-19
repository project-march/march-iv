cmake_minimum_required(VERSION 2.8.3)
project(march_description)

find_package(catkin REQUIRED COMPONENTS xacro)

catkin_package(CFG_EXTRAS ${PROJECT_NAME}-extras.cmake)

include(cmake/${PROJECT_NAME}-extras.cmake)

file(GLOB xacro_files urdf/*.xacro)
build_and_install_xacro_files(${xacro_files})

install(DIRECTORY urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
