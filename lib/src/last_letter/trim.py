"""Trim a multicopter recipe at hover.

The tool loads the recipe, removes the ground reaction and the wind, and runs
the trimmer through a mixer. It prints the trim point. The console script
`last-letter-trim` runs it. `last-letter-linearize` runs the same function
before it linearizes, so both tools give the same trim point.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__copyright__ = "Copyright 2026, George Zogopoulos"

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

from . import paths
from .control import ControlMap
from .control import CopterControlMap
from .control import TrimPoint
from .control import TrimTarget
from .control import Trimmer
from .control import build_mixer
from .control import load_model_config
from .simulation import Aircraft

# The mixer of the ArduPilot SITL default quad, the default mixer file.
STOCK_SITL_MIXER = Path(__file__).parent / "data" / "stock_sitl_mixer.yaml"

# The labels of the four controls of a copter control map.
CONTROL_LABELS = ("roll", "pitch", "yaw", "throttle")


@dataclass
class TrimResult:
    """What one trim computed and the objects that hold it.

    The aircraft sits at the trim point with its components settled. The
    control map is the one that was used in trimming the aircraft.
    """

    recipe: str
    aircraft: Aircraft
    control_map: ControlMap
    mixer_block: dict
    trim: TrimPoint


def stock_mixer_block() -> dict:
    """Return the mixer/ block of the stock SITL quad as a dictionary."""
    return yaml.safe_load(STOCK_SITL_MIXER.read_text())


def euler(trim: TrimPoint) -> tuple:
    """Return the roll, pitch and yaw of the trim attitude in rad."""
    angles = trim.state.get_euler()
    return float(angles.roll), float(angles.pitch), float(angles.yaw)


def trim_recipe(
    recipe: str,
    models_root: Optional[Path] = None,
    mixer_file: Optional[Path] = None,
) -> TrimResult:
    """Trim a multicopter recipe at hover through a mixer.

    :param recipe: The name of the recipe under aircraft/.
    :param models_root: The models root that holds the recipe. None means
        paths.models_path().
    :param mixer_file: A YAML file with a mixer/ block and its mixerType. None
        means the stock SITL mixer.
    :raises RuntimeError: if the trim does not converge.
    """
    source_root = Path(models_root) if models_root else paths.models_path()
    mixer_path = Path(mixer_file) if mixer_file else STOCK_SITL_MIXER
    mixer_block = yaml.safe_load(mixer_path.read_text())

    # The ground reaction gives the trimmer a second solution, where the legs
    # carry the weight at zero throttle, so it goes. The trim is for still air.
    config = yaml.safe_load(load_model_config(recipe, str(source_root)))
    config["dynamics"]["ground"]["groundReactionType"] = 0
    config["env"]["windRef"] = 0.0
    aircraft = Aircraft(recipe)
    aircraft.initialize(yaml.safe_dump(config))

    mixer = build_mixer(yaml.safe_dump(mixer_block))
    control_map = CopterControlMap(mixer, aircraft)
    trim = Trimmer().trim(aircraft, control_map, TrimTarget())
    if not trim.converged:
        raise RuntimeError(
            f"The trim of {recipe} did not converge. The residual is "
            f"{trim.residual:.3e} and the NLopt code is {trim.return_code}."
        )
    return TrimResult(
        recipe=recipe,
        aircraft=aircraft,
        control_map=control_map,
        mixer_block=mixer_block,
        trim=trim,
    )


def print_trim(trim: TrimPoint, file=None) -> None:
    """Print the trim point as a block with a `Trim point` title.

    :param file: The stream to print to. None means the stdout at call time.
    """
    file = file or sys.stdout
    phi, theta, psi = euler(trim)
    print("Trim point", file=file)
    print(
        f"  position NED  {np.array2string(np.asarray(trim.state.position), precision=3)} m",
        file=file,
    )
    print(
        f"  attitude      roll {phi:.3e}, pitch {theta:.3e}, yaw {psi:.3e} rad",
        file=file,
    )
    print(
        "  controls      "
        + ", ".join(
            f"{label} {float(u):.6f}" for label, u in zip(CONTROL_LABELS, trim.controls)
        ),
        file=file,
    )
    print(
        f"  residual      {trim.residual:.3e} after {trim.evaluations} evaluations",
        file=file,
    )


def main() -> int:
    """Entry point of the `last-letter-trim` console script."""
    parser = argparse.ArgumentParser(
        prog="last-letter-trim",
        description=(
            "Trim a multicopter recipe at hover and print the trim point. "
            "The trim is the one that last-letter-linearize uses."
        ),
    )
    parser.add_argument("recipe", help="The recipe, for example ardupilot_copter.")
    parser.add_argument(
        "--mixer",
        type=Path,
        help=(
            "A YAML file with the mixer/ block and its mixerType. The default is "
            "the stock ArduPilot SITL quad."
        ),
    )
    parser.add_argument(
        "--models-root",
        type=Path,
        help="The models root of the recipe. The default is the models path.",
    )
    args = parser.parse_args()

    try:
        result = trim_recipe(args.recipe, args.models_root, args.mixer)
    except (RuntimeError, FileNotFoundError) as error:
        print(error, file=sys.stderr)
        return 1
    print_trim(result.trim)
    return 0


if __name__ == "__main__":
    sys.exit(main())
