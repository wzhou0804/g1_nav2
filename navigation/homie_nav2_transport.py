"""Loopback velocity link between the ROS environment and the CPU policy process."""
import json
import math
import socket
import time
import uuid

DEFAULT_PORT = 17651
COMMAND_TIMEOUT = 0.35  # Same lease as the existing real-navigation velocity gate.
STATUS_TIMEOUT = 0.6  # Same response timeout as the factory velocity RPC.
ODOMETRY_TIMEOUT = 0.5  # Existing ROS bridge odometry freshness requirement.


def decode(data):
    message = json.loads(data)
    if message['version'] != 1 or not isinstance(message['session'], str):
        raise ValueError('Unsupported local velocity message')
    if type(message['sequence']) is not int or not math.isfinite(message['stamp']):
        raise ValueError('Invalid local velocity sequence/time')
    return message


class HomieNavClient:
    """ROS side: send gated velocity and read the policy's readiness reply."""
    def __init__(self, port=DEFAULT_PORT):
        self.destination = ('127.0.0.1', port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', 0))
        self.socket.setblocking(False)
        self.session = uuid.uuid4().hex
        self.sequence = 0
        self.ack_sequence = -1
        self.status = {}
        self.last_ack = -math.inf

    def send(self, velocity=(0., 0., 0.), enabled=False, measured_velocity=None, measurement_stamp=None, stop_immediately=False):
        self.sequence += 1
        message = dict(version=1, session=self.session, sequence=self.sequence,
                       stamp=time.monotonic(), enabled=bool(enabled), velocity=list(velocity),
                       measured_velocity=measured_velocity, measurement_stamp=measurement_stamp,
                       stop_immediately=bool(stop_immediately))
        self.socket.sendto(json.dumps(message, allow_nan=False).encode(), self.destination)

    def poll(self):
        while True:
            try:
                data, source = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            if source != self.destination:
                continue
            try:
                message = decode(data)
                if (message['session'] != self.session or
                        not self.ack_sequence < message['sequence'] <= self.sequence or
                        not 0 <= time.monotonic() - message['stamp'] <= STATUS_TIMEOUT or
                        type(message['active']) is not bool or type(message['ready']) is not bool or
                        not isinstance(message['phase'], str)):
                    continue
            except (KeyError, ValueError, TypeError, UnicodeDecodeError):
                continue
            self.ack_sequence = message['sequence']
            self.last_ack = message['stamp']
            self.status = {key: message[key] for key in ('ready', 'active', 'phase')}

    def ready(self):
        return (self.status.get('ready') is True and
                0 <= time.monotonic() - self.last_ack <= STATUS_TIMEOUT)

    def close(self):
        try:
            self.send(enabled=False)
        finally:
            self.socket.close()


class HomieNavCommands:
    """Policy side: a disabled packet claims the link; expired input becomes zero."""
    def __init__(self, port=DEFAULT_PORT, require_feedback=False):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', port))
        self.socket.setblocking(False)
        self.port = self.socket.getsockname()[1]
        self.session = None
        self.sequence = -1
        self.last_received = -math.inf
        self.command = (0., 0., 0.)
        self.enabled = False
        self.requested = False
        self.stop_immediately = False
        self.require_feedback = require_feedback
        self.measurement = None
        self.measurement_stamp = None

    def measured_velocity(self):
        if (self.measurement is None or self.measurement_stamp is None or
                not 0 <= time.monotonic() - self.measurement_stamp < ODOMETRY_TIMEOUT):
            return None
        return self.measurement

    def poll(self, active=False, phase='waiting', ready=None):
        while True:
            try:
                data, source = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            now = time.monotonic()
            try:
                message = decode(data)
                velocity = tuple(float(v) for v in message['velocity'])
                if (len(velocity) != 3 or not all(math.isfinite(v) for v in velocity) or
                        type(message['enabled']) is not bool or
                        type(message.get('stop_immediately',False)) is not bool or
                        not 0 <= now - message['stamp'] <= COMMAND_TIMEOUT):
                    continue
                if message['session'] != self.session:
                    if message['enabled'] or now - self.last_received <= COMMAND_TIMEOUT:
                        continue
                    self.session, self.sequence = message['session'], -1
                if message['sequence'] <= self.sequence:
                    continue
            except (KeyError, ValueError, TypeError, UnicodeDecodeError):
                continue
            self.sequence = message['sequence']
            self.last_received = message['stamp']
            self.measurement = None
            self.measurement_stamp = None
            try:
                measured = tuple(float(v) for v in message['measured_velocity'])
                measured_stamp = float(message['measurement_stamp'])
                if (len(measured) == 3 and all(math.isfinite(v) for v in measured) and
                        math.isfinite(measured_stamp) and 0 <= now-measured_stamp < ODOMETRY_TIMEOUT):
                    self.measurement = measured
                    self.measurement_stamp = measured_stamp
            except (KeyError, TypeError, ValueError):
                pass
            feedback_ready = not self.require_feedback or self.measured_velocity() is not None
            self.requested = message['enabled'] and feedback_ready
            self.enabled = active and self.requested
            self.stop_immediately = message.get('stop_immediately',False)
            self.command = velocity if self.enabled and feedback_ready and not self.stop_immediately else (0., 0., 0.)
            reply = {key: message[key] for key in ('version', 'session', 'sequence', 'stamp')}
            reply.update(active=bool(active), ready=bool(active if ready is None else ready) and feedback_ready, phase=phase)
            self.socket.sendto(json.dumps(reply).encode(), source)
        if time.monotonic() - self.last_received > COMMAND_TIMEOUT:
            self.requested = False
        if self.require_feedback and self.measured_velocity() is None:
            self.requested = False
        if not active or not self.enabled or not self.requested or (self.require_feedback and self.measured_velocity() is None):
            self.enabled = False
            self.command = (0., 0., 0.)
        return self.command

    def close(self):
        self.socket.close()
