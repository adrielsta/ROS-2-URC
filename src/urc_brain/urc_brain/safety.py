"""Pure, testable command arbitration. All timestamps use monotonic seconds.

This is an application interlock, NOT an independent hardware emergency stop.
"""
from dataclasses import dataclass
from enum import Enum
import math


class Mode(str, Enum):
    HOLD = 'hold'
    AUTONOMY = 'autonomy'
    TELEOP = 'teleop'
    ESTOP = 'estop'


@dataclass(frozen=True)
class Limits:
    max_linear: float = 0.5
    max_angular: float = 0.8
    command_timeout: float = 0.4
    odom_timeout: float = 1.0
    scan_timeout: float = 1.0
    heartbeat_timeout: float = 1.0

    def __post_init__(self):
        if any(not math.isfinite(x) or x <= 0 for x in vars(self).values()):
            raise ValueError('Every safety limit must be finite and positive')


class CommandGate:
    def __init__(self, limits=Limits()):
        self.limits = limits
        self.mode = Mode.HOLD
        self.reason = 'startup_disarmed'
        self.last_odom = self.last_scan = self.last_heartbeat = -math.inf
        self.command = (0.0, 0.0)
        self.command_at = -math.inf
        self.stopped = False

    @staticmethod
    def fresh(now, stamp, timeout):
        return math.isfinite(now) and 0 <= now - stamp <= timeout

    def healthy(self, now):
        return (self.fresh(now, self.last_odom, self.limits.odom_timeout)
                and self.fresh(now, self.last_scan, self.limits.scan_timeout))

    def stop(self, reason='operator_stop'):
        if self.mode != Mode.ESTOP:
            self.mode = Mode.HOLD
        self.reason = reason
        self.command = (0.0, 0.0)
        self.command_at = -math.inf

    def estop(self):
        self.mode = Mode.ESTOP
        self.stop('estop_latched')

    def reset_estop(self, now):
        if self.mode != Mode.ESTOP:
            return False, 'No emergency stop is latched'
        if not self.healthy(now) or not self.stopped:
            return False, 'Fresh sensors and measured zero velocity required'
        self.mode = Mode.HOLD
        self.stop('estop_reset_disarmed')
        return True, self.reason

    def arm(self, mode, now):
        if mode not in (Mode.AUTONOMY, Mode.TELEOP):
            return False, 'Unsupported drive mode'
        if self.mode == Mode.ESTOP:
            return False, 'Emergency stop is latched'
        if not self.healthy(now) or not self.stopped:
            return False, 'Fresh sensors and measured zero velocity required'
        if mode == Mode.TELEOP and not self.fresh(
                now, self.last_heartbeat, self.limits.heartbeat_timeout):
            return False, 'Teleoperation requires a C2 heartbeat'
        # Never replay a command from the previous operating mode.
        self.command = (0.0, 0.0)
        self.command_at = -math.inf
        self.mode, self.reason = mode, 'armed'
        return True, self.reason

    def receive(self, source, linear, angular, now):
        if source != self.mode or source not in (Mode.AUTONOMY, Mode.TELEOP):
            return
        if not all(math.isfinite(v) for v in (linear, angular, now)):
            self.stop('invalid_command')
            return
        self.command = (
            max(-self.limits.max_linear, min(self.limits.max_linear, linear)),
            max(-self.limits.max_angular, min(self.limits.max_angular, angular)),
        )
        self.command_at = now

    def output(self, now):
        if self.mode not in (Mode.AUTONOMY, Mode.TELEOP):
            return 0.0, 0.0
        if not self.healthy(now):
            self.stop('sensor_timeout')
            return 0.0, 0.0
        if self.mode == Mode.TELEOP and not self.fresh(
                now, self.last_heartbeat, self.limits.heartbeat_timeout):
            self.stop('c2_timeout')
            return 0.0, 0.0
        if not self.fresh(now, self.command_at, self.limits.command_timeout):
            # Nav2 may legitimately pause while planning. Zero, never hold last speed.
            return 0.0, 0.0
        return self.command

