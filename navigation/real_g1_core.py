"""Transport-independent guards for the original G1 locomotion service."""
import math
from dataclasses import dataclass


@dataclass
class VelocityGate:
    permitted: bool = False
    armed: bool = False
    last_command: float = -math.inf
    command: tuple = (0.0, 0.0, 0.0)
    output: tuple = (0.0, 0.0, 0.0)
    limits: tuple = (0.15, 0.06, 0.25)
    acceleration: tuple = (0.20, 0.15, 0.40)
    timeout: float = 0.35
    reason: str = 'motion disabled'

    def update(self, values, now):
        if not all(math.isfinite(v) for v in values):
            self.disarm('nonfinite velocity')
            return
        self.command = tuple(max(-lim, min(lim, v)) for v, lim in zip(values, self.limits))
        self.last_command = now

    def arm(self, healthy, now):
        if not self.permitted or not healthy:
            return False
        self.armed = True
        # Never replay a command received before arming.
        self.last_command = -math.inf
        self.command = self.output = (0.0, 0.0, 0.0)
        self.reason = 'armed; awaiting a new command'
        return True

    def disarm(self, reason):
        self.armed = False
        self.reason = reason
        self.command = self.output = (0.0, 0.0, 0.0)

    def step(self, now, healthy, dt=0.1):
        if not self.armed:
            return None  # No RPC at all, including zero, while disarmed.
        if not healthy:
            self.disarm('stale sensors, clock fault, model mismatch or FSM/RPC fault')
            return (0.0, 0.0, 0.0)
        if now - self.last_command > self.timeout:
            self.output = (0.0, 0.0, 0.0)
            self.reason = 'command watchdog: stop'
            return self.output
        self.output = tuple(old + max(-a * dt, min(a * dt, v - old))
                            for old, v, a in zip(self.output, self.command, self.acceleration))
        self.reason = 'following command'
        return self.output


@dataclass
class RobotClock:
    offset_ns: int | None = None
    fault: bool = False
    last_robot_ns: int | None = None

    def observe(self, robot_ns, host_ns):
        if robot_ns <= 0:
            self.fault = True
            return
        if self.offset_ns is None:
            self.offset_ns = host_ns - robot_ns
        if (self.last_robot_ns is not None and robot_ns < self.last_robot_ns - 10_000_000
                or abs(robot_ns + self.offset_ns - host_ns) > 750_000_000):
            self.fault = True  # Latched: restart explicitly to establish a new epoch.
        self.last_robot_ns = robot_ns

    def convert(self, robot_ns):
        if self.offset_ns is None or self.fault:
            return None
        return robot_ns + self.offset_ns
