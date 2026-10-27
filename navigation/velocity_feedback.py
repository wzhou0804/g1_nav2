"""Bounded velocity feedback before a learned locomotion policy.

The outer loop runs at 50 Hz and uses measured yaw-frame velocity. Navigation can
slew normal zero requests to rest. No joint targets or odometry are altered.
"""
import numpy as np


class VelocityFeedback:
    def __init__(self, dt=.02):
        self.dt = dt
        self.limit = np.array([.55, .55, 1.57])
        self.kp = np.array([1., 1., 1.2])
        self.ki = np.array([.9, .9, 1.5])
        self.integral_limit = np.array([.35, .35, 1.2])
        self.acceleration = np.array([.8, .8, 2.5])
        self.feedforward = np.zeros(3)
        self.smooth_stop = False
        self.reset()

    def reset(self):
        self.integral = np.zeros(3)
        self.filtered_velocity = np.zeros(3)
        self.previous_command = np.zeros(3)
        self.previous_desired = np.zeros(3)

    def update(self, desired, measured, dt=None):
        dt = self.dt if dt is None else dt
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('Feedback dt must be positive and finite')
        desired = np.asarray(desired, dtype=float)
        measured = np.asarray(measured, dtype=float)
        if desired.shape != (3,) or measured.shape != (3,):
            raise ValueError('Expected vx, vy, wz vectors')
        if not np.isfinite(desired).all() or not np.isfinite(measured).all():
            self.reset()
            return np.zeros(3)
        if np.all(np.abs(desired) <= np.array([.005, .005, .01])):
            previous = self.previous_command.copy()
            self.reset()
            if self.smooth_stop:
                self.previous_command = previous + np.clip(-previous, -self.acceleration*dt, self.acceleration*dt)
            return self.previous_command.copy()
        self.filtered_velocity += dt / (.20 + dt) * (measured-self.filtered_velocity)
        self.integral[desired * self.previous_desired < 0] = 0.
        error = desired-self.filtered_velocity
        feedforward = self.feedforward*np.sign(desired)
        proposed = np.clip(self.integral + self.ki*error*dt,
                           -self.integral_limit, self.integral_limit)
        unsaturated = desired+feedforward+self.kp*error+proposed
        # Conditional integration: do not accumulate error into a saturated output.
        keep = (np.abs(unsaturated) <= self.limit) | (unsaturated*error < 0)
        self.integral = np.where(keep, proposed, self.integral)
        target = np.clip(desired+feedforward+self.kp*error+self.integral,-self.limit,self.limit)
        change = self.acceleration*dt
        self.previous_command += np.clip(target-self.previous_command,-change,change)
        self.previous_desired = desired.copy()
        return self.previous_command.copy()
