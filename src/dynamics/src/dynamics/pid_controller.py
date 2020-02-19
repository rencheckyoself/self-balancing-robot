"""
Contains a class to perform PID control.

This file is based on the Arduino PID library written by Brett Beauregard
http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
"""

class Controller(object):
    """
    Class to define a PID controller and compute the nessissary controls

    Input:
        kp: (double) proportional gain, must be positive
        ki: (double) integral gain, must be positive
        kd: (double) differential gain. must be positive
        dt: (double) a constant timestep
        set_point: (double) the value the cotroller will drive the system to
        direction: (int) either 1 or -1 to set the direction
    """
    def __init__(self, kp, ki, kd, dt, set_point, direction):

        self.kp = abs(kp)
        self.ki = abs(ki)
        self.kd = abs(kd)
        self.timestep = dt
        self.set_point = set_point
        self.direction = direction

        self.max_lim = 10000
        self.min_lim = -10000

        self.error_sum = 0
        self.prev_error = 0
        self.prev_input = 0

        self._mod_gains()

    def compute_control(self, input):
        """
        Function to compute a control.

        Inputs:
            input: (double) the value to base the control on

        Output:
            control: (double) the control command

        Notes:
            - Uses "Derivative on Measurment" instead of "Derivative on Error"
        """

        # Compute error terms
        error = self.set_point - input
        self.error_sum += error
        d_input = (input - self.prev_input)
        self.prev_input = input
        self.prev_error = error

        # Compute Control
        control = self.direction * (self.kp * error + self.ki * self.error_sum - self.kd * d_input)

        control = max(min(control, self.max_lim), self.min_lim)

        return control

    def _mod_gains(self):
        self.ki *= self.timestep
        self.kd /= self.timestep

    def change_output_lims(self, minlim, maxlim):
        """
        Function to change the control output limits

        Inputs:
            minlim: (double) the minimum allowable control limit
            maxlim: (double) the maximum allowable control limit
        """

        self.max_lim = maxlim
        self.min_lim = minlim

    def change_direction(self, direction):
        """
        Function to change the direction of the PID output

        Input:
            direction: (int) either 1 or -1 to set the direction
        """
        if direction == 1 or direction == -1:
            self.direction = direction

    def change_setpoint(self, new_val):
        """
        Function to modify the set point

        Input:
            new_val: (double) the new set point
        """
        self.set_point = new_val
