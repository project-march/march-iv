#!/usr/bin/env bash

build_passed () {
    notify "Your local build has passed!"
}


build_failed () {
    notify "$1"
    exit 1
}

notify () {
    if [[ -x "$(command -v notify-send)" ]]; then
        notify-send -i $PWD/walking.png "$1"
    fi
}

# Enable test coverage
catkin config --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug

# Build entire workspace
catkin build --summarize  --no-notify|| build_failed "Could not build workspace"
source devel/setup.bash

# Catkin lint
catkin lint -W2 --pkg march_description || build_failed "Catkin lint failed in march_description"
catkin lint -W2 --pkg march_gait_selection || build_failed "Catkin lint failed in march_gait_selection"
catkin lint -W2 --pkg march_sound_scheduler || build_failed "Catkin lint failed in march_sound_scheduler"
catkin lint -W2 --pkg march_gait_scheduler || build_failed "Catkin lint failed in march_gait_scheduler"
catkin lint -W2 --pkg march_launch || build_failed "Catkin lint failed in march_launch"
catkin lint -W2 --pkg march_safety || build_failed "Catkin lint failed in march_safety"
catkin lint -W2 --pkg march_shared_resources || build_failed "Catkin lint failed in march_shared_resources"

# Roslint
# march_description and march_launch do not need to be roslinted as they don't contain any code.
catkin build --no-deps --verbose march_gait_selection --no-notify --catkin-make-args roslint || build_failed "Roslint failed in march_gait_selection"
catkin build --no-deps --verbose march_gait_scheduler --no-notify --catkin-make-args roslint || build_failed "Roslint failed in march_gait_scheduler"
catkin build --no-deps --verbose march_sound_scheduler --no-notify --catkin-make-args roslint || build_failed "Roslint failed in march_sound_scheduler"
catkin build --no-deps --verbose march_safety --no-notify --catkin-make-args roslint || build_failed "Roslint failed in march_safety"
catkin build --no-deps --verbose march_shared_resources --no-notify --catkin-make-args roslint || build_failed "Roslint failed in march_shared_resources"

# Run all tests in the workspace, including roslaunch-checks if they exist
catkin build --summarize --catkin-make-args run_tests && catkin_test_results build/ --verbose || build_failed "Tests failed"
