import math
import pytest
from urc_brain.safety import CommandGate, Limits, Mode


def ready(now=10):
    gate = CommandGate()
    gate.last_odom = gate.last_scan = gate.last_heartbeat = now
    gate.stopped = True
    return gate


def test_startup_disarmed_and_wrong_source_cannot_drive():
    gate = ready()
    gate.receive(Mode.AUTONOMY, 1, 1, 10)
    assert gate.output(10) == (0, 0)
    assert gate.arm(Mode.AUTONOMY, 10)[0]
    gate.receive(Mode.TELEOP, 1, 1, 10)
    assert gate.output(10) == (0, 0)


def test_clamps_and_times_out_last_command():
    gate = ready()
    gate.arm(Mode.AUTONOMY, 10)
    gate.receive(Mode.AUTONOMY, 9, -9, 10)
    assert gate.output(10) == (0.5, -0.8)
    assert gate.output(10.41) == (0, 0)


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_bad_command_disarms(bad):
    gate = ready()
    gate.arm(Mode.AUTONOMY, 10)
    gate.receive(Mode.AUTONOMY, bad, 0, 10)
    assert gate.output(10) == (0, 0)
    assert gate.mode == Mode.HOLD


def test_sensor_failure_requires_explicit_rearm():
    gate = ready()
    gate.arm(Mode.AUTONOMY, 10)
    gate.receive(Mode.AUTONOMY, 0.3, 0, 11.1)
    assert gate.output(11.1) == (0, 0)
    assert gate.reason == 'sensor_timeout'
    gate.last_odom = gate.last_scan = 12
    gate.receive(Mode.AUTONOMY, 0.3, 0, 12)
    assert gate.output(12) == (0, 0)


def test_link_loss_differs_between_teleop_and_autonomy():
    for mode in [Mode.TELEOP, Mode.AUTONOMY]:
        gate = ready()
        assert gate.arm(mode, 10)[0]
        gate.last_odom = gate.last_scan = 20
        gate.receive(mode, 0.3, 0, 20)
        assert gate.output(20) == ((0, 0) if mode == Mode.TELEOP else (0.3, 0))


def test_estop_is_latched_and_reset_never_rearms():
    gate = ready()
    gate.arm(Mode.AUTONOMY, 10)
    gate.estop()
    assert not gate.arm(Mode.AUTONOMY, 10)[0]
    gate.stopped = False
    assert not gate.reset_estop(10)[0]
    gate.stopped = True
    assert gate.reset_estop(10)[0]
    assert gate.mode == Mode.HOLD
    assert gate.output(10) == (0, 0)


def test_mode_switch_discards_previous_command():
    gate = ready()
    gate.arm(Mode.TELEOP, 10)
    gate.receive(Mode.TELEOP, 0.4, 0, 10)
    assert gate.arm(Mode.AUTONOMY, 10)[0]
    assert gate.output(10) == (0, 0)


def test_reject_unhealthy_moving_or_unconnected_arming():
    gate = ready()
    gate.stopped = False
    assert not gate.arm(Mode.AUTONOMY, 10)[0]
    gate.stopped = True
    gate.last_heartbeat = -math.inf
    assert not gate.arm(Mode.TELEOP, 10)[0]
    assert not gate.arm(Mode.AUTONOMY, 30)[0]


def test_backwards_time_fails_closed():
    gate = ready()
    gate.arm(Mode.AUTONOMY, 10)
    gate.receive(Mode.AUTONOMY, 0.2, 0, 10)
    assert gate.output(9) == (0, 0)


def test_invalid_limits_rejected():
    with pytest.raises(ValueError):
        Limits(command_timeout=-1)

