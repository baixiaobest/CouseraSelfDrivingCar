import math
import numpy as np

class PIDController:
    def __init__(self, Kp, Ki, Kd, min, max):
        self.Kp = float(Kp)
        self.Ki = float(Ki)
        self.Kd = float(Kd)
        self.min = float(min)
        self.max = float(max)
        self.prev_error = 0
        self.integrator = 0

    def update(self, reference, current, delta_t, feedforward_signal=0.0):
        error = reference - current
        p_term = self.Kp * error
        d_term = self.Kd * (error - self.prev_error) / delta_t
        i_term = self.Ki * self.integrator

        total_signal = p_term + d_term + i_term + feedforward_signal

        saturate = False

        # If output saturates, clamp the signal.
        if total_signal < self.min or total_signal > self.max:
            total_signal = max(self.min, min(total_signal, self.max))
            saturate = True

        # If integrator saturates and error and signal are the same sign
        # (which means integrator is making the situation worse), stop integrating,
        # and remove integrator term.
        if saturate and np.sign(error) * np.sign(total_signal) > 0:
            pass
        else:
            self.integrator += error * delta_t

        return total_signal
