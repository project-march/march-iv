
class Setpoint(object):
    """Base class to define the setpoints of a subgait."""

    digits = 4

    def __init__(self, time, position, velocity):
        self.time = round(time, self.digits)
        self.position = round(position, self.digits)
        self.velocity = round(velocity, self.digits)

    def __repr__(self):
        return 'Time: %s, Position: %s, Velocity: %s' % (self.time, self.position, self.velocity)
