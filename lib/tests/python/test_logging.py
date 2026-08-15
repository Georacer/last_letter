#!/usr/bin/env python3
"""Tests for the data-logging feature from the Python side.

These verify that the Python controls (last_letter.enable_logging /
disable_logging / log_frame) drive C++ logging for a bare Component, with no
UavModel and no logger object passed around.
"""

__authors__ = ["George Zogopoulos"]

import gc

import last_letter
from last_letter import aerodynamics
from last_letter.environment import EnvironmentData
from last_letter.utils.math import UnitQuaternion
from last_letter.utils.math import Vector3
from last_letter.utils.uav import UavState


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


def test_logging_api_is_exposed():
    """The Python logging API is the on/off toggle plus the frame trigger."""
    assert hasattr(last_letter, "enable_logging")
    assert hasattr(last_letter, "disable_logging")
    assert hasattr(last_letter, "log_frame")


def test_bare_component_logs_to_mcap(tmp_path):
    """A standalone airfoil (no UavModel) is logged by log_frame()."""
    from last_letter.utils import log

    airfoil = aerodynamics.SimpleDrag("test_airfoil")
    airfoil.initialize(AIRFOIL_DESC)

    state = UavState(Vector3(), UnitQuaternion(), Vector3(20, 0, 0), Vector3())
    env = EnvironmentData()

    n_steps = 10
    path = tmp_path / "aero.mcap"
    last_letter.enable_logging(str(path))
    for _ in range(n_steps):
        airfoil.update_local_state(state, env)
        airfoil.calc_model()
        last_letter.log_frame()
    last_letter.disable_logging()

    assert path.exists(), "enable_logging did not create the MCAP file"
    assert path.stat().st_size > 0, "MCAP file is empty; no snapshots were written"

    # Decode the MCAP and check the schema/rows match what the component logs.
    frames = log.load_log(str(path))
    assert "test_airfoil" in frames, f"channel missing; got {list(frames)}"
    df = frames["test_airfoil"]
    assert len(df) == n_steps, f"expected {n_steps} snapshots, got {len(df)}"
    # With no time given, log_frame() stamps an index counting from 0 (in ns).
    assert list(df.index) == [i * 1e-9 for i in range(n_steps)]
    # Aerodynamics adds airdata on top of the base wrench_sum.
    for col in ("airdata/airspeed", "airdata/alpha", "airdata/beta"):
        assert col in df.columns, f"missing series {col!r}; have {list(df.columns)}"
    assert any(
        c.startswith("wrench_sum/aero/force") for c in df.columns
    ), f"no wrench_sum aero force series; have {list(df.columns)}"


def test_load_log_merged_and_csv(tmp_path):
    """load_log_merged and to_csv produce a wide, channel-prefixed table."""
    import pandas as pd

    from last_letter.utils import log

    airfoil = aerodynamics.SimpleDrag("test_airfoil")
    airfoil.initialize(AIRFOIL_DESC)
    state = UavState(Vector3(), UnitQuaternion(), Vector3(20, 0, 0), Vector3())
    env = EnvironmentData()

    path = tmp_path / "aero.mcap"
    last_letter.enable_logging(str(path))
    for _ in range(5):
        airfoil.update_local_state(state, env)
        airfoil.calc_model()
        last_letter.log_frame()
    last_letter.disable_logging()

    merged = log.load_log_merged(str(path))
    assert len(merged) == 5
    assert "test_airfoil/airdata/alpha" in merged.columns

    csv_path = tmp_path / "aero.csv"
    log.to_csv(str(path), str(csv_path))
    assert csv_path.exists()
    reloaded = pd.read_csv(csv_path)
    assert len(reloaded) == 5


def test_no_file_written_when_logging_disabled(tmp_path):
    """With logging off, stepping and log_frame() must not create any file."""
    airfoil = aerodynamics.SimpleDrag("test_airfoil")
    airfoil.initialize(AIRFOIL_DESC)

    state = UavState(Vector3(), UnitQuaternion(), Vector3(20, 0, 0), Vector3())
    env = EnvironmentData()

    path = tmp_path / "should_not_exist.mcap"
    for _ in range(10):
        airfoil.update_local_state(state, env)
        airfoil.calc_model()
        last_letter.log_frame()  # A harmless no-op while logging is off.

    assert not path.exists()


def test_two_components_share_one_timestamp(tmp_path):
    """One log_frame() puts every channel on one timestamp.

    This is what keeps a multi-component log rectangular: load_log_merged()
    outer-joins the channels on their timestamp, so channels stamped
    independently would come back as one half-empty row each.
    """
    from last_letter.utils import log

    airfoil_a = aerodynamics.SimpleDrag("airfoil_a")
    airfoil_a.initialize(AIRFOIL_DESC)
    airfoil_b = aerodynamics.SimpleDrag("airfoil_b")
    airfoil_b.initialize(AIRFOIL_DESC)

    state = UavState(Vector3(), UnitQuaternion(), Vector3(20, 0, 0), Vector3())
    env = EnvironmentData()

    path = tmp_path / "two_components.mcap"
    last_letter.enable_logging(str(path))
    for airfoil in (airfoil_a, airfoil_b):
        airfoil.update_local_state(state, env)
        airfoil.calc_model()
    last_letter.log_frame()
    last_letter.disable_logging()

    frames = log.load_log(str(path))
    assert set(frames) == {"airfoil_a", "airfoil_b"}, f"got {sorted(frames)}"
    assert frames["airfoil_a"].index[0] == frames["airfoil_b"].index[0]

    merged = log.load_log_merged(str(path))
    assert len(merged) == 1, f"expected one row, got {len(merged)}"
    assert not merged.isna().any().any(), "ragged join; the channels disagree on time"
    assert "airfoil_a/airdata/alpha" in merged.columns
    assert "airfoil_b/airdata/alpha" in merged.columns


def test_explicit_time_stamps_the_frame(tmp_path):
    """log_frame(t) stamps the frame with the simulation time it is given."""
    from last_letter.utils import log

    airfoil = aerodynamics.SimpleDrag("test_airfoil")
    airfoil.initialize(AIRFOIL_DESC)

    state = UavState(Vector3(), UnitQuaternion(), Vector3(20, 0, 0), Vector3())
    env = EnvironmentData()

    path = tmp_path / "timed.mcap"
    last_letter.enable_logging(str(path))
    for t in (0.0, 0.5):
        airfoil.update_local_state(state, env)
        airfoil.calc_model()
        last_letter.log_frame(t)
    last_letter.disable_logging()

    merged = log.load_log_merged(str(path))
    assert list(merged.index) == [0.0, 0.5]


def test_destroyed_component_leaves_the_frame_set(tmp_path):
    """A component drops its channel when it dies.

    The values a component registers are pointers into it, so a channel left
    behind by a destroyed one would make the next log_frame() read freed
    memory. The surviving component must keep logging normally afterwards.
    """
    from last_letter.utils import log

    airfoil_a = aerodynamics.SimpleDrag("airfoil_a")
    airfoil_a.initialize(AIRFOIL_DESC)
    airfoil_b = aerodynamics.SimpleDrag("airfoil_b")
    airfoil_b.initialize(AIRFOIL_DESC)

    state = UavState(Vector3(), UnitQuaternion(), Vector3(20, 0, 0), Vector3())
    env = EnvironmentData()

    path = tmp_path / "destroyed.mcap"
    last_letter.enable_logging(str(path))
    for airfoil in (airfoil_a, airfoil_b):
        airfoil.update_local_state(state, env)
        airfoil.calc_model()
    last_letter.log_frame()

    # pybind holds the instance in a unique_ptr owned by Python, so CPython's
    # refcounting runs the C++ destructor as soon as the last name goes. The
    # loop variable above is still one of them.
    del airfoil, airfoil_b
    gc.collect()

    airfoil_a.update_local_state(state, env)
    airfoil_a.calc_model()
    last_letter.log_frame()
    last_letter.disable_logging()

    frames = log.load_log(str(path))
    assert len(frames["airfoil_a"]) == 2, "the survivor stopped logging"
    assert len(frames["airfoil_b"]) == 1, "a destroyed component kept logging"
