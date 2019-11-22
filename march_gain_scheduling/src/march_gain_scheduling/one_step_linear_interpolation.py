#!/usr/bin/env python

import rospy


def interpolate(current_gains, needed_gains, gradient, delta_t):
    if len(current_gains) != len(needed_gains):
        rospy.logerr("needed_gains and current_gains not equal length")
        return current_gains
    next_gains = [0] * len(current_gains)
    for i in range(len(current_gains)):
        if current_gains[i] > needed_gains[i]:
            next_gains[i] = current_gains[i] - gradient * delta_t
            if next_gains[i] < needed_gains[i]:
                next_gains[i] = needed_gains[i]
        else:
            next_gains[i] = current_gains[i] + gradient * delta_t
            if next_gains[i] > needed_gains[i]:
                next_gains[i] = needed_gains[i]

    return next_gains
