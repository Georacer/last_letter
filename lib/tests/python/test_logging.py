#!/usr/bin/env python3
"""Tests for the data-logging feature from the Python side.

These verify that the Python on/off toggle (last_letter_lib.enable_logging /
disable_logging) drives C++ self-logging for a bare Component, with no UavModel
and no logger object passed around.
"""

__authors__ = ["George Zogopoulos"]

import last_letter_lib
from last_letter_lib import aerodynamics
from last_letter_lib.environment import EnvironmentData
from last_letter_lib.utils.math import UnitQuaternion
from last_letter_lib.utils.math import Vector3
from last_letter_lib.utils.uav import UavState


AIRFOIL_DESC = """
s: 1
b: 2
c: 0.5
c_L_0: 0.3269
c_L_alpha: 3.85
c_D_0: 0.0322
c_D_alpha: 0.0037
c_Y_beta: 0.1
c_l_0: 0
c_l_deltaa: 1
c_m_0: 0.1
c_m_alpha: -0.5
c_m_deltae: 1
c_n_0: 0
c_n_beta: 0.1
c_n_deltar: 0.1
alpha_stall: 0.26
inertial:
    mass: 1.8
    tensor:
        j_xx: 0.8244
        j_yy: 1.135
        j_zz: 1.759
        j_xz: 0.1204
"""


def test_enable_disable_logging_are_exposed():
    """The only logging API in Python is the on/off toggle."""
    assert hasattr(last_letter_lib, "enable_logging")
    assert hasattr(last_letter_lib, "disable_logging")


def test_bare_component_logs_to_mcap(tmp_path):
    """A standalone airfoil (no UavModel) self-logs once logging is enabled."""
    from last_letter_lib.utils import log

    airfoil = aerodynamics.SimpleDrag("test_airfoil")
    airfoil.initialize(AIRFOIL_DESC)

    state = UavState(Vector3(), UnitQuaternion(), Vector3(20, 0, 0), Vector3())
    env = EnvironmentData()

    n_steps = 10
    path = tmp_path / "aero.mcap"
    last_letter_lib.enable_logging(str(path))
    for _ in range(n_steps):
        airfoil.update_local_state(state, env)
        airfoil.calc_model()
    last_letter_lib.disable_logging()

    assert path.exists(), "enable_logging did not create the MCAP file"
    assert path.stat().st_size > 0, "MCAP file is empty; no snapshots were written"

    # Decode the MCAP and check the schema/rows match what the component logs.
    frames = log.load_log(str(path))
    assert "test_airfoil" in frames, f"channel missing; got {list(frames)}"
    df = frames["test_airfoil"]
    assert len(df) == n_steps, f"expected {n_steps} snapshots, got {len(df)}"
    # Aerodynamics adds airdata on top of the base wrench_sum.
    for col in ("airdata/airspeed", "airdata/alpha", "airdata/beta"):
        assert col in df.columns, f"missing series {col!r}; have {list(df.columns)}"
    assert any(
        c.startswith("wrench_sum/aero/force") for c in df.columns
    ), f"no wrench_sum aero force series; have {list(df.columns)}"


def test_load_log_merged_and_csv(tmp_path):
    """load_log_merged and to_csv produce a wide, channel-prefixed table."""
    import pandas as pd

    from last_letter_lib.utils import log

    airfoil = aerodynamics.SimpleDrag("test_airfoil")
    airfoil.initialize(AIRFOIL_DESC)
    state = UavState(Vector3(), UnitQuaternion(), Vector3(20, 0, 0), Vector3())
    env = EnvironmentData()

    path = tmp_path / "aero.mcap"
    last_letter_lib.enable_logging(str(path))
    for _ in range(5):
        airfoil.update_local_state(state, env)
        airfoil.calc_model()
    last_letter_lib.disable_logging()

    merged = log.load_log_merged(str(path))
    assert len(merged) == 5
    assert "test_airfoil/airdata/alpha" in merged.columns

    csv_path = tmp_path / "aero.csv"
    log.to_csv(str(path), str(csv_path))
    assert csv_path.exists()
    reloaded = pd.read_csv(csv_path)
    assert len(reloaded) == 5


def test_no_file_written_when_logging_disabled(tmp_path):
    """With logging off, stepping a component must not create any file."""
    airfoil = aerodynamics.SimpleDrag("test_airfoil")
    airfoil.initialize(AIRFOIL_DESC)

    state = UavState(Vector3(), UnitQuaternion(), Vector3(20, 0, 0), Vector3())
    env = EnvironmentData()

    path = tmp_path / "should_not_exist.mcap"
    for _ in range(10):
        airfoil.update_local_state(state, env)
        airfoil.calc_model()

    assert not path.exists()
