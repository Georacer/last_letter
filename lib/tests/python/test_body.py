#!/usr/bin/env python3
"""Tests for the body components."""

__authors__ = ["George Zogopoulos"]

import pytest

import last_letter
from last_letter import body
from last_letter.environment import EnvironmentData
from last_letter.utils.math import UnitQuaternion
from last_letter.utils.math import Vector3
from last_letter.utils.uav import Inputs
from last_letter.utils.uav import UavState

LINEAR_COPTER_DESC = """
roll:
  Llat: 305.3
  wlg: 174.0
pitch:
  Mlon: 307.3
  wlg: 174.8
yaw:
  Nped: 8.18
  wlg: 166.9
vertical:
  Zcoll: -26.5
  wlg: 333.5
"""


def test_linear_copter_logs_lag_states(tmp_path):
    """A bare LinearCopter logs its four lag states."""
    from last_letter.utils import log

    copter = body.LinearCopter("linear_copter")
    copter.initialize(LINEAR_COPTER_DESC)
    copter.set_input(Inputs([0.1, -0.2, 0.3, 0.5]))
    state = UavState(Vector3(), UnitQuaternion(), Vector3(), Vector3())
    env = EnvironmentData()

    n_steps = 20
    path = tmp_path / "linear_copter.mcap"
    last_letter.enable_logging(str(path))
    for _ in range(n_steps):
        copter.update_local_state(state, env)
        copter.calc_model()
        last_letter.log_frame()
    last_letter.disable_logging()

    frames = log.load_log(str(path))
    assert "linear_copter" in frames, f"channel missing; got {list(frames)}"
    df = frames["linear_copter"]
    assert len(df) == n_steps
    final = df.iloc[-1]
    assert final["d_rol"] == pytest.approx(copter.d_rol)
    assert final["d_pit"] == pytest.approx(copter.d_pit)
    assert final["d_yaw"] == pytest.approx(copter.d_yaw)
    assert final["d_thr"] == pytest.approx(copter.d_thr)
    # Each lag state moves toward its input and does not overshoot it.
    for name, target in (("rol", 0.1), ("pit", -0.2), ("yaw", 0.3), ("thr", 0.5)):
        series = df[f"d_{name}"]
        assert 0 < series.iloc[0] / target < series.iloc[-1] / target <= 1.0
