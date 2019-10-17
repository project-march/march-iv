cmake_minimum_required(VERSION 2.8.3)
project(march_rqt_launch_menu)

find_package(catkin REQUIRED COMPONENTS
    roslint
)

catkin_python_setup()
catkin_package()

install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch resource
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB_RECURSE lintfiles "src/*.py")
roslint_python(${lintfiles})
