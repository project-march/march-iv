import rospy

class UnequalLengthError(Exception):
    print "current_gains and needed_gains do not have the same length"
    pass