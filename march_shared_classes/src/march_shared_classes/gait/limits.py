class Limits(object):

    def __init__(self, lower, upper, velocity, effort=None, k_position=None, k_velocity=None):
        self.lower = lower
        self.upper = upper
        self.velocity = velocity
        self.effort = effort
        self.k_position = k_position
        self.k_velocity = k_velocity
