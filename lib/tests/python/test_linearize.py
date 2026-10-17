"""Tests for the recipe writer of last-letter-linearize."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from last_letter.control import CopterControlMap
from last_letter.control import Trimmer
from last_letter.control import TrimTarget
from last_letter.control import build_mixer
from last_letter.control import load_model_config
from last_letter.linearize import linearize_recipe
from last_letter.linearize import stock_mixer_block
from last_letter.simulation import Aircraft
from last_letter.utils.uav import Inputs

TEST_MODELS_ROOT = Path(__file__).resolve().parents[1] / "cpp" / "test_parameters"
SOURCE = "ardupilot_copter"
NAME = "ardupilot_copter_from_trim"


@pytest.fixture(scope="module")
def written(tmp_path_factory):
    """Write the linear recipe of the stock quad into a temporary models root."""
    out = tmp_path_factory.mktemp("models")
    result = linearize_recipe(SOURCE, TEST_MODELS_ROOT, None, NAME, out)
    return out, result


def _aircraft(name, root, airborne):
    """Build an aircraft from a recipe. Airborne removes the ground and the wind."""
    config = yaml.safe_load(load_model_config(name, str(root)))
    if airborne:
        config["dynamics"]["ground"]["groundReactionType"] = 0
        config["env"]["windRef"] = 0.0
    aircraft = Aircraft(name)
    aircraft.initialize(yaml.safe_dump(config))
    return aircraft, config


def test_writes_the_recipe_files(written):
    out, result = written
    recipe_dir = out / "aircraft" / NAME
    assert result.recipe_dir == recipe_dir
    for file_name in (
        "body.yaml",
        "aerodynamics.yaml",
        "propulsion.yaml",
        "ground.yaml",
        "init.yaml",
        "randomizer.yaml",
    ):
        assert (recipe_dir / file_name).is_file()

    body = yaml.safe_load((recipe_dir / "body.yaml").read_text())
    assert body["nBodies"] == 1
    linear = body["body1"]
    assert linear["bodyType"] == 1
    assert linear["yaw"]["Npedp"] == 0.0
    for axis in ("roll", "pitch", "yaw", "vertical"):
        assert linear[axis]["wlg"] == 0.0
    assert linear["vertical"]["u_hover"] == pytest.approx(result.trim.controls[3])
    assert linear["mixer"] == stock_mixer_block()
    assert linear["inertial"]["mass"] == pytest.approx(3.0)
    assert yaml.safe_load((recipe_dir / "aerodynamics.yaml").read_text()) == {
        "nWings": 0
    }
    assert yaml.safe_load((recipe_dir / "propulsion.yaml").read_text()) == {
        "nMotors": 0
    }
    # The output keeps the ground reaction of the source, not the override.
    source_dir = TEST_MODELS_ROOT / "aircraft" / SOURCE
    for file_name in ("ground.yaml", "init.yaml", "randomizer.yaml"):
        assert (recipe_dir / file_name).read_text() == (
            source_dir / file_name
        ).read_text()
    assert (recipe_dir / "body.yaml").read_text().startswith("# ")


def test_written_recipe_flies(written):
    """The written recipe loads and holds the hover for 100 steps."""
    out, result = written
    aircraft, _ = _aircraft(NAME, out, airborne=False)
    z_init = aircraft.state.position[2]
    aircraft.set_input(Inputs([0.0, 0.0, 0.0, result.coefficients.u_hover]))
    for _ in range(100):
        aircraft.step()

    assert np.all(np.isfinite(aircraft.state.position))
    assert np.all(np.isfinite(aircraft.state.velocity_linear))
    assert np.all(np.isfinite(aircraft.state.velocity_angular))
    assert abs(aircraft.state.position[2] - z_init) < 0.05
    assert np.linalg.norm(aircraft.state.velocity_angular) < 1e-6


def test_roll_doublet_matches_the_source(written):
    """A roll doublet from the trim gives the same peak roll rate on both models."""
    out, result = written
    source, config = _aircraft(SOURCE, TEST_MODELS_ROOT, airborne=True)
    mixer = build_mixer(yaml.safe_dump(stock_mixer_block()))
    control_map = CopterControlMap(mixer, source)
    point = Trimmer().trim(source, control_map, TrimTarget())
    assert point.converged

    linear, _ = _aircraft(NAME, out, airborne=True)
    source.set_state(point.state)
    linear.set_state(point.state)

    dt = config["world"]["deltaT"]
    steps = int(round(1.0 / dt))
    amplitude = 0.005
    trim_controls = np.asarray(point.controls)
    p_source = []
    p_linear = []
    for i in range(steps):
        roll = amplitude if i < steps // 2 else -amplitude
        controls = trim_controls.copy()
        controls[0] += roll
        source.set_input(control_map.to_input(controls))
        linear.set_input(Inputs(list(controls)))
        source.step()
        linear.step()
        p_source.append(source.state.velocity_angular[0])
        p_linear.append(linear.state.velocity_angular[0])

    peak_source = np.max(np.abs(p_source))
    peak_linear = np.max(np.abs(p_linear))
    print(f"Peak roll rate: source {peak_source:.6f}, linear {peak_linear:.6f} rad/s")
    assert peak_source > 0.1
    assert abs(peak_linear - peak_source) < 0.05 * peak_source
