import math
class PID:

    def __init__(self, kp=0.0, ki=0.0, kd=0.0,
                 output_limits=(None, None),
                 integral_limits=(None, None),
                 deadzone=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min, self.output_max = output_limits
        self.integral_min, self.integral_max = integral_limits
        self.deadzone = deadzone

        self.setpoint = 0.0
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_update = True

 
    def set_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_setpoint(self, target):
        self.setpoint = target

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_update = True

    def compute(self, measurement, dt, error=None):
   
        if error is None:
            error = self.setpoint - measurement

        #  Target deadzone: kill jitter when error is basically zero 
        if abs(error) < self.deadzone:
            self._prev_error = 0.0
            # Still let existing integral contribute (holds position) but
            # do not grow it further and do not add derivative kick.
            output = self.kp * 0.0 + self.ki * self._integral
            return self._clamp_output(output)

        # Conditional integration / zero-crossing reset 
        # If the error just flipped sign, the system has crossed the target
        # (overshot). Any accumulated integral from the previous side is
        # stale and would fight the correction, so wipe it.
        if self._prev_error != 0.0 and (error > 0) != (self._prev_error > 0):
            self._integral = 0.0

        self._integral += error * dt
        self._integral = self._clamp(self._integral, self.integral_min, self.integral_max)


        if dt > 0.0 and not self._first_update:
            derivative = (error - self._prev_error) / dt
        else:
            derivative = 0.0
        self._first_update = False

        #  PID sum
        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)
        self._prev_error = error

        return self._clamp_output(output)

    
    def _clamp(self, value, vmin, vmax):
        if vmin is not None:
            value = max(vmin, value)
        if vmax is not None:
            value = min(vmax, value)
        return value

    def _clamp_output(self, value):
        return self._clamp(value, self.output_min, self.output_max)

    @staticmethod
    def normalize_angle(angle):
        #Wraps any angle (radians) to the range [-pi, pi].
        return math.atan2(math.sin(angle), math.cos(angle))
