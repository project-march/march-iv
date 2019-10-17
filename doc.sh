#!/usr/bin/env bash

packages="march_description march_gait_scheduler march_sound_scheduler march_gait_selection march_launch march_safety march_state_machine march_rqt_launch_menu"

mkdir -p docs/html
for package in $packages; do
    catkin document $package --no-deps
    mv docs/$package/html docs/html/$package
done
