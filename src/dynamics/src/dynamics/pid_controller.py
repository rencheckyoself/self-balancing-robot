""" Contains a class to perform PID control """

class Controller(object):
    """
    Class to define a PID controller and compute the nessissary controls

    Input:
        kp: (double) proportional gain
        ki: (double) integral gain
        kd: (double) differential gain
        dt: (double) a constant timestep
        set_point: (double) the value the cotroller will drive the system to
    """
    def __init__(self, kp, ki, kd, dt, set_point):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.timestep = dt
        self.set_point = set_point

        self.error = 0

    def compute_control():
        pass

    def change_setpoint(new_val):
        """
        Function to modify the set point

        Input:
            new_val: (double) the new set point
        """
        self.setpoint = new_val
