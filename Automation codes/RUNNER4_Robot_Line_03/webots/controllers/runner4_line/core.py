"""Simulator-independent perception and planar motion. SI units except error_mm."""
import math
from dataclasses import dataclass

SENSOR_Y = tuple((4 - i) * 0.010 for i in range(9))  # S1 left, S9 right
SENSOR_X = 0.085
BODY_Z = 0.121
START = (-0.40, -0.45, 0.0)
CHANNELS = {'FL': (0, 1, 2), 'FR': (3, 4, 5),
            'BL': (10, 11, 14), 'BR': (6, 8, 9)}


@dataclass
class Config:
    speed: float = 0.045  # m/s; one lap is about 99 seconds
    kp: float = 0.032     # (rad/s)/mm; NOT the firmware's degree/mm gain
    kd: float = 0.0015   # (rad/s)/(mm/s)
    max_yaw_rate: float = 0.65
    alpha: float = 0.5
    threshold: float = 0.45
    lost_timeout: float = 3.0


class LineFollower:
    def __init__(self, config=None):
        self.cfg = config or Config()
        self.filtered = None
        self.previous_error = None
        self.lost_time = 0.0
        self.stopped = False
        self.junction_count = 0
        self.error_mm = 0.0
        self.state = 'STARTING'

    def update(self, samples, dt):
        if len(samples) != 9 or dt <= 0:
            raise ValueError('Expected nine reflectances and positive dt')
        values = [max(0.0, min(1.0, v)) for v in samples]
        a = self.cfg.alpha
        self.filtered = values if self.filtered is None else [
            a*v + (1-a)*old for v, old in zip(values, self.filtered)]
        weights = [v if v >= self.cfg.threshold else 0.0 for v in self.filtered]
        if self.stopped:
            self.state = 'SAFE STOP - press R'
            return 0.0, 0.0
        if sum(weights) == 0:
            self.lost_time += dt
            self.previous_error = None
            self.junction_count = 0
            # Stop translation immediately; bounded search toward last sighting.
            if self.lost_time >= self.cfg.lost_timeout:
                self.stopped = True
                self.state = 'SAFE STOP - press R'
                return 0.0, 0.0
            self.state = 'LINE LOST - searching'
            turn = math.copysign(0.18, self.error_mm) if abs(self.error_mm) > 1 else 0.0
            return 0.0, turn
        self.lost_time = 0.0
        self.error_mm = 1000 * sum(y*w for y, w in zip(SENSOR_Y, weights)) / sum(weights)
        derivative = 0 if self.previous_error is None else (self.error_mm-self.previous_error)/dt
        self.previous_error = self.error_mm
        yaw = self.cfg.kp*self.error_mm + self.cfg.kd*derivative
        yaw = max(-self.cfg.max_yaw_rate, min(self.cfg.max_yaw_rate, yaw))
        self.junction_count = self.junction_count + 1 if sum(w > 0 for w in weights) >= 6 else 0
        self.state = 'WIDE LINE' if self.junction_count >= 3 else 'FOLLOWING'
        speed = self.cfg.speed / (1.5 if abs(self.error_mm) > 20 else 1)
        return speed, yaw


def integrate(pose, speed, yaw_rate, dt):
    """Explicit planar surrogate, not foot-contact dynamics."""
    x, y, heading = pose
    middle = heading + yaw_rate*dt/2
    return x + speed*math.cos(middle)*dt, y + speed*math.sin(middle)*dt, heading + yaw_rate*dt


def gait_targets(time, moving, yaw_rate=0):
    """Smooth diagonal trot visualisation; local joint zero is model standing."""
    targets = {}
    for leg, side, offset in [('FL', 1, 0), ('FR', -1, math.pi),
                              ('BL', 1, math.pi), ('BR', -1, 0)]:
        phase = time*2*math.pi/1.2 + offset
        amplitude = math.radians(20)*max(0.35, min(1.65, 1-side*yaw_rate))
        swing = math.sin(phase) if moving else 0
        lift = max(0, math.cos(phase)) if moving else 0
        targets[leg] = (-side*amplitude*swing, -side*math.radians(20)*lift,
                        side*math.radians(8)*lift)
    return targets
