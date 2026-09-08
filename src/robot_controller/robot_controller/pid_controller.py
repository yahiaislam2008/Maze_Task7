import math
class PIDController:
  
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        setpoint: float = 0.0,
        output_limits: tuple = (None, None),
        integral_limits: tuple = None,
        deadzone: float = 0.0,
        angle_wrap: bool = False,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.setpoint = setpoint
        self.output_limits = output_limits
        # Anti-windup clamp defaults to the output limits if not specified.
        self.integral_limits = (
            integral_limits if integral_limits is not None else output_limits
        )
        self.deadzone = deadzone
        self.angle_wrap = angle_wrap

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_error_valid = False


    def set_setpoint(self, target: float):
        self.setpoint = target

    def set_gains(self, kp: float = None, ki: float = None, kd: float = None):
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd

    def reset(self):
       
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_error_valid = False

    def compute(self, current_value: float, dt: float) -> float:
        
        error = self.setpoint - current_value

        # Angle normalization 
        # For yaw/heading control, wrap error into [-pi, pi] so the robot
        # always takes the shortest rotation instead of spinning the long
        # way around when the raw subtraction crosses the +/-pi boundary.
        if self.angle_wrap:
            error = math.atan2(math.sin(error), math.cos(error))

        #  Target deadzone 
        # If the error is negligible, output nothing and don't let the
        # integral term keep creeping -> stops motor jitter at the target.
        if abs(error) < self.deadzone:
            self._prev_error = 0.0
            self._prev_error_valid = True
            return 0.0

        # Conditional integration / zero-crossing reset
        # If the error just changed sign (we overshot the setpoint), the
        # old integral term is no longer relevant and can cause the robot
        # to overshoot again on the way back. Reset it at the crossing.
        if self._prev_error_valid and (error * self._prev_error) < 0:
            self._integral = 0.0

        # Only integrate while dt is sane (avoids a huge spike on the
        # very first call or after a long pause).
        if dt > 0.0:
            self._integral += error * dt

        #  Integral anti-windup (clamping) 
        lo, hi = self.integral_limits
        if lo is not None:
            self._integral = max(lo, self._integral)
        if hi is not None:
            self._integral = min(hi, self._integral)

        #  Derivative 

        derivative = 0.0
        if dt > 0.0 and self._prev_error_valid:
            derivative = (error - self._prev_error) / dt

        self._prev_error = error
        self._prev_error_valid = True

        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        # Control output clamping 
        lo, hi = self.output_limits
        if lo is not None:
            output = max(lo, output)
        if hi is not None:
            output = min(hi, output)

        return output



def _simulate():
    pid = PIDController(
        kp=1.2,
        ki=0.05,
        kd=0.3,
        setpoint=10.0,      # target
        output_limits=(-5.0, 5.0),
        deadzone=0.02,
    )

    current_state = 0.0   # starting state variable
    dt = 0.1               # simulated time step (s)

    for step in range(60):
        error = pid.setpoint - current_state
        output = pid.compute(current_state, dt)

        # Apply Physics: mimic mechanical movement
        current_state += output * dt

        print(
            f'Step {step:02d} | error={error:7.3f} | '
            f'output={output:7.3f} | state={current_state:7.3f}'
        )

        if abs(error) < pid.deadzone:
            print('System stabilized within deadzone.')
            break


if __name__ == '__main__':
    _simulate()
