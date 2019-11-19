class Joint:

    def __init__(self, name, limits, setpoints, duration):
        self.name = name
        self.limits = limits
        self.setpoints = setpoints
        self.duration = duration

    def get_setpoint(self, index):
        return self.setpoints[index]

    def set_setpoints(self, setpoints):
        self.setpoints = setpoints

    def get_setpoints_unzipped(self):
        time = []
        position = []
        velocity = []

        for i in range(0, len(self.setpoints)):
            time.append(self.setpoints[i].time)
            position.append(self.setpoints[i].position)
            velocity.append(self.setpoints[i].velocity)

        return time, position, velocity
