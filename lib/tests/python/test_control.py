"""Tests for the trimmer and the linearizer bindings."""

from pathlib import Path

import yaml

from last_letter.control import CopterControlMap
from last_letter.control import Trimmer
from last_letter.control import TrimTarget
from last_letter.control import build_mixer
from last_letter.control import load_model_config
from last_letter.simulation import Aircraft

TEST_MODELS_ROOT = str(Path(__file__).resolve().parents[1] / "cpp" / "test_parameters")


def test_trim_ardupilot_copter():
    """Trim the stock quad through the stock ArduPilot mixer."""
    config = yaml.safe_load(load_model_config("ardupilot_copter", TEST_MODELS_ROOT))
    config["dynamics"]["ground"]["groundReactionType"] = 0
    config["env"]["windRef"] = 0.0
    aircraft = Aircraft("ardupilot_copter")
    aircraft.initialize(yaml.safe_dump(config))

    linear = yaml.safe_load(
        load_model_config("ardupilot_copter_linear", TEST_MODELS_ROOT)
    )
    mixer = build_mixer(yaml.safe_dump(linear["dynamics"]["body"]["body1"]["mixer"]))
    assert mixer.mixer_type() == 1
    control_map = CopterControlMap(mixer, aircraft)

    point = Trimmer().trim(aircraft, control_map, TrimTarget())
    assert point.converged
    assert len(point.controls) == 4
    assert abs(point.controls[3] - 0.37) < 0.02
    assert all(abs(c) < 1e-6 for c in point.controls[:3])
