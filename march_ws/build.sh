#!/usr/bin/env bash

build_passed () {
    notify "Your local build has passed!"
}


build_failed () {
    notify "$1"
    exit 1
}

notify () {
    if [ -x "$(command -v notify-send)" ]; then
        notify-send -i $PWD/walking.png "$1"
    fi
}

catkin_make || build_failed "Catkin_make failed"


# cpplinter.
catkin_make roslint_march_state_machine_service || build_failed "ros_lint failed in march_state_machine_service"
catkin_make roslint_march_control || build_failed "ros_lint failed in march_control"
catkin_make roslint_march_state_machine || build_failed "ros_lint failed in march_control"

# Catkin lint, fail on errors only. TODO remove missing_directory.
catkin_lint src/*/ --ignore missing_directory --ignore literal_project_name --ignore missing_install_target --explain -W2 || build_failed "catkin_lint failed"


# Run the tests, ensuring the path is set correctly.
source devel/setup.bash || exit 1
catkin_make run_tests && catkin_test_results || build_failed "Tests failed"

for directory in src/*/launch
do
    rosrun roslaunch roslaunch-check $directory || build_failed "roslaunch-check failed in directory $directory"
done

build_passed
