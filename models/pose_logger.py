class PoseLogger:
    """
    A pose logger class used to record pose info of a rover.
    """
    def __init__(self, rover):
        self.rover = rover
        self.x_pose = []
        self.y_pose = []
        self.velocity = []
        self.termination_time = None

    def log_pose(self):
        """
        Record pose.
        """
        self.x_pose.append(self.rover.pose[0])
        self.y_pose.append(self.rover.pose[1])

    def log_velocity(self):
        """
        Record velocity.
        """
        self.velocity.append(self.rover.control[0])

    def log_termination_time(self):
        """
        Record termination time.
        """
        self.termination_time = self.rover.termination_time
