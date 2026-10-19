"""Tests for the trim tool of last-letter-trim."""

import io
from pathlib import Path

import numpy as np
import pytest

from last_letter.linearize import linearize_recipe
from last_letter.trim import main
from last_letter.trim import print_trim
from last_letter.trim import stock_mixer_block
from last_letter.trim import trim_recipe

TEST_MODELS_ROOT = Path(__file__).resolve().parents[1] / "cpp" / "test_parameters"
RECIPE = "ardupilot_copter"


@pytest.fixture(scope="module")
def trimmed():
    """Trim the stock quad through the stock mixer."""
    return trim_recipe(RECIPE, TEST_MODELS_ROOT)


def test_trim_converges_at_hover(trimmed):
    trim = trimmed.trim
    assert trimmed.recipe == RECIPE
    assert trimmed.mixer_block == stock_mixer_block()
    assert trim.converged
    assert trim.residual < 1e-4
    controls = np.asarray(trim.controls)
    assert controls[:3] == pytest.approx(0.0, abs=1e-6)
    assert abs(controls[3] - 0.37) < 0.02
    assert np.linalg.norm(trim.state.velocity_linear) < 1e-6
    assert np.linalg.norm(trim.state.velocity_angular) < 1e-6


def test_trim_matches_linearize(trimmed, tmp_path):
    """The trim of last-letter-linearize is the same trim point."""
    result = linearize_recipe(RECIPE, TEST_MODELS_ROOT, None, "from_trim", tmp_path)
    expected = trimmed.trim
    actual = result.trim
    assert np.asarray(actual.controls) == pytest.approx(
        np.asarray(expected.controls), abs=0.0
    )
    assert np.asarray(actual.state.position) == pytest.approx(
        np.asarray(expected.state.position), abs=0.0
    )
    assert np.asarray(actual.state.velocity_linear) == pytest.approx(
        np.asarray(expected.state.velocity_linear), abs=0.0
    )
    assert np.asarray(actual.state.velocity_angular) == pytest.approx(
        np.asarray(expected.state.velocity_angular), abs=0.0
    )
    assert actual.residual == expected.residual
    assert actual.evaluations == expected.evaluations


def test_aircraft_sits_at_the_trim(trimmed):
    """The returned aircraft holds the trim state and settled components."""
    aircraft = trimmed.aircraft
    assert np.asarray(aircraft.state.position) == pytest.approx(
        np.asarray(trimmed.trim.state.position), abs=0.0
    )
    assert np.asarray(aircraft.state.velocity_linear) == pytest.approx(
        np.asarray(trimmed.trim.state.velocity_linear), abs=0.0
    )


def test_print_trim_lists_the_controls(trimmed):
    stream = io.StringIO()
    print_trim(trimmed.trim, stream)
    text = stream.getvalue()
    assert text.startswith("Trim point\n")
    assert "position NED" in text
    assert "controls      roll " in text
    assert f"throttle {float(trimmed.trim.controls[3]):.6f}" in text
    assert f"after {trimmed.trim.evaluations} evaluations" in text


def test_main_prints_the_trim(monkeypatch, capsys):
    monkeypatch.setattr(
        "sys.argv",
        ["last-letter-trim", RECIPE, "--models-root", str(TEST_MODELS_ROOT)],
    )
    assert main() == 0
    # capsys sees the Python prints only. The C++ model builder writes to the
    # file descriptor and so stays out of this capture.
    out = capsys.readouterr().out
    assert out.startswith("Trim point\n")
    assert "throttle" in out


def test_main_reports_a_missing_recipe(monkeypatch, capsys):
    monkeypatch.setattr(
        "sys.argv",
        ["last-letter-trim", "no_such_recipe", "--models-root", str(TEST_MODELS_ROOT)],
    )
    assert main() == 1
    assert capsys.readouterr().err.strip() != ""
