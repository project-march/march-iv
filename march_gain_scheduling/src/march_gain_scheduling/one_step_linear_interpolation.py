import rospy

from march_gain_scheduling import UnequalLengthError


def interpolate(current_gains, needed_gains, gradient, delta_t):
    if len(current_gains) != len(needed_gains):
        raise UnequalLengthError
    next_gains = [0] * len(current_gains)
    for i in range(len(current_gains)):
        if current_gains[i] > needed_gains[i]:
            next_gains[i] = max(needed_gains[i], current_gains[i] - gradient * delta_t)
        else:
            next_gains[i] = min(needed_gains[i], current_gains[i] - gradient * delta_t)

    return next_gains
