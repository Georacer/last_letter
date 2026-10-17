"""Write a linear copter recipe from a nonlinear multicopter recipe.

The tool trims the source recipe at hover, linearizes it about the trim point
and writes a recipe with one LinearCopter body. The SITL bridge flies the
result with no change. The console script `last-letter-linearize` runs it.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__copyright__ = "Copyright 2026, George Zogopoulos"

import argparse
import datetime
import shutil
import sys
from dataclasses import dataclass
from dataclasses import field
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

from . import paths
from .control import CopterControlMap
from .control import LinearCopterCoefficients
from .control import LinearModel
from .control import Linearizer
from .control import LinearizerOptions
from .control import TrimPoint
from .control import TrimTarget
from .control import Trimmer
from .control import build_mixer
from .control import load_model_config
from .simulation import Aircraft

# The mixer of the ArduPilot SITL default quad, the default mixer file.
STOCK_SITL_MIXER = Path(__file__).parent / "data" / "stock_sitl_mixer.yaml"

# The files the output recipe copies from the source recipe as they are.
_COPIED_FILES = ("ground.yaml", "init.yaml", "randomizer.yaml")
# The files at the models root that load_model_config needs.
_ROOT_FILES = ("world.yaml", "environment.yaml")

_ROWS = ("a_x", "a_y", "a_z", "alpha_x", "alpha_y", "alpha_z")
_COLUMNS = {
    "A_v": ("u", "v", "w"),
    "A_w": ("p", "q", "r"),
    "A_att": ("phi", "theta", "psi"),
    "B": ("roll", "pitch", "yaw", "throttle"),
}
# The entries of the Jacobians that the LinearCopter keeps, as (row, column).
# Every other entry is a coupling term that the recipe drops.
_KEPT = {
    "A_v": {(0, 0), (1, 1), (2, 2), (3, 1), (4, 0)},
    "A_w": {(3, 0), (4, 1), (5, 2)},
    "A_att": set(),
    "B": {(0, 1), (1, 0), (2, 3), (3, 0), (4, 1), (5, 2)},
}


@dataclass
class DroppedTerm:
    """One entry of a Jacobian that the LinearCopter recipe does not hold."""

    matrix: str
    row: str
    column: str
    value: float

    def label(self) -> str:
        """Return the derivative as text, for example d alpha_x / d q."""
        return f"d {self.row} / d {self.column}"


@dataclass
class LinearizeResult:
    """What one run of the tool computed and where it wrote it."""

    recipe_dir: Path
    trim: TrimPoint
    model: LinearModel
    coefficients: LinearCopterCoefficients
    dropped: list = field(default_factory=list)


def _euler(trim: TrimPoint) -> tuple:
    """Return the roll, pitch and yaw of the trim attitude in rad."""
    euler = trim.state.get_euler()
    return float(euler.roll), float(euler.pitch), float(euler.yaw)


def stock_mixer_block() -> dict:
    """Return the mixer/ block of the stock SITL quad as a dictionary."""
    return yaml.safe_load(STOCK_SITL_MIXER.read_text())


def dropped_terms(model: LinearModel, threshold: float = 1e-9) -> list:
    """Return the Jacobian entries that the recipe drops, largest first.

    :param model: The linear model.
    :param threshold: Leave out the entries with a smaller magnitude.
    """
    terms = []
    for name, columns in _COLUMNS.items():
        matrix = np.asarray(getattr(model, name))
        for i, row in enumerate(_ROWS):
            for j, column in enumerate(columns):
                if (i, j) in _KEPT[name]:
                    continue
                value = float(matrix[i, j])
                if abs(value) > threshold:
                    terms.append(DroppedTerm(name, row, column, value))
    terms.sort(key=lambda term: -abs(term.value))
    return terms


def _body_yaml(
    coefficients: LinearCopterCoefficients, inertial, mixer: dict, header: str
) -> str:
    """Return the text of body.yaml for one LinearCopter."""
    tensor = np.asarray(inertial.tensor)
    c = coefficients
    body = {
        "nBodies": 1,
        "body1": {
            "name": "linear_copter",
            "bodyType": 1,
            "inertial": {
                "mass": float(inertial.mass),
                "tensor": {
                    "j_xx": float(tensor[0, 0]),
                    "j_yy": float(tensor[1, 1]),
                    "j_zz": float(tensor[2, 2]),
                    # The Component reads the product of inertia with the
                    # opposite sign of the tensor entry. Adding 0.0 turns a -0.0
                    # into 0.0.
                    "j_xz": float(-tensor[0, 2]) + 0.0,
                },
            },
            "chanRoll": 0,
            "chanPitch": 1,
            "chanYaw": 2,
            "chanThrottle": 3,
            "roll": {
                "Yv": c.Yv,
                "Ylat": c.Ylat,
                "Lv": c.Lv,
                "Lp": c.Lp,
                "Llat": c.Llat,
                "wlg": 0.0,
            },
            "pitch": {
                "Xu": c.Xu,
                "Xlon": c.Xlon,
                "Mu": c.Mu,
                "Mq": c.Mq,
                "Mlon": c.Mlon,
                "wlg": 0.0,
            },
            "yaw": {"Nr": c.Nr, "Nped": c.Nped, "Npedp": 0.0, "wlg": 0.0},
            "vertical": {
                "Zw": c.Zw,
                "Zcoll": c.Zcoll,
                "wlg": 0.0,
                "u_hover": c.u_hover,
            },
            "mixer": mixer,
        },
    }
    text = yaml.safe_dump(body, sort_keys=False, default_flow_style=False)
    return header + "\n" + text


def _header(recipe: str, trim: TrimPoint, options: LinearizerOptions, tensor) -> str:
    """Return the comment block at the top of body.yaml."""
    phi, theta, psi = _euler(trim)
    controls = ", ".join(f"{float(u):.6f}" for u in trim.controls)
    position = ", ".join(f"{float(x):.3f}" for x in trim.state.position)
    lines = [
        f"A linear model of the recipe {recipe}, written by last-letter-linearize.",
        "Body type 1 is LinearCopter. The input is the ideal command into the",
        "mixer of the mixer/ block, roll, pitch and yaw in -1 to 1 and throttle",
        "in 0 to 1.",
        "",
        f"Date: {datetime.date.today().isoformat()}.",
        f"Trim point: position NED [{position}] m, roll {phi:.3e} rad,",
        f"pitch {theta:.3e} rad, yaw {psi:.3e} rad, still air.",
        f"Trim controls [roll, pitch, yaw, throttle]: [{controls}].",
        f"Trim residual: {trim.residual:.3e}, after {trim.evaluations} evaluations.",
        f"Step sizes: velocity {options.step_velocity} m/s, rate "
        f"{options.step_rate} rad/s,",
        f"attitude {options.step_attitude} rad, control {options.step_control}.",
        "The derivatives leave out gravity, the Coriolis term and the",
        "gyroscopic term, because the Component and Kinematics add them.",
        "",
        "Every wlg is 0, which means no lag state. A fit of the lag is a later",
        "addition. Npedp is 0, so Nped holds all of the yaw control power.",
        "The inertial block is the lumped mass and tensor of the source about its",
        "center of gravity.",
    ]
    if abs(tensor[0, 1]) > 1e-12 or abs(tensor[1, 2]) > 1e-12:
        lines.append(
            "The source tensor has j_xy or j_yz, which this block cannot hold."
        )
    return "\n".join(f"# {line}".rstrip() for line in lines) + "\n"


def linearize_recipe(
    recipe: str,
    models_root: Optional[Path] = None,
    mixer_file: Optional[Path] = None,
    name: Optional[str] = None,
    out: Optional[Path] = None,
) -> LinearizeResult:
    """Trim and linearize a multicopter recipe and write a LinearCopter recipe.

    :param recipe: The name of the source recipe under aircraft/.
    :param models_root: The models root that holds the source recipe. None
        means paths.models_path().
    :param mixer_file: A YAML file with a mixer/ block and its mixerType. None
        means the stock SITL mixer.
    :param name: The name of the output recipe. None means <recipe>_linear.
    :param out: The models root to write into. None means paths.models_path().
        When it lacks world.yaml or environment.yaml, the tool copies them from
        the source root, so the output root loads on its own.
    :raises RuntimeError: if the trim does not converge.
    """
    source_root = Path(models_root) if models_root else paths.models_path()
    out_root = Path(out) if out else paths.models_path()
    name = name or f"{recipe}_linear"
    mixer_path = Path(mixer_file) if mixer_file else STOCK_SITL_MIXER
    mixer_block = yaml.safe_load(mixer_path.read_text())

    # The ground reaction gives the trimmer a second solution, where the legs
    # carry the weight at zero throttle, so it goes. The trim and the
    # derivatives are for still air.
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

    options = LinearizerOptions()
    options.include_gravity = False
    options.include_coriolis = False
    options.include_gyroscopic = False
    model = Linearizer(options).linearize(aircraft, control_map, trim)
    coefficients = Linearizer.to_linear_copter(model)
    # The trimmer leaves the aircraft at the trim point.
    inertial = aircraft.dynamics.inertial
    tensor = np.asarray(inertial.tensor)

    recipe_dir = out_root / "aircraft" / name
    recipe_dir.mkdir(parents=True, exist_ok=True)
    header = _header(recipe, trim, options, tensor)
    (recipe_dir / "body.yaml").write_text(
        _body_yaml(coefficients, inertial, mixer_block, header)
    )
    (recipe_dir / "aerodynamics.yaml").write_text(
        "# The linear model in body.yaml lumps the aerodynamics of the airframe.\n"
        "\nnWings: 0\n"
    )
    (recipe_dir / "propulsion.yaml").write_text(
        "# The linear model in body.yaml lumps the propulsion of the airframe.\n"
        "\nnMotors: 0\n"
    )
    source_dir = source_root / "aircraft" / recipe
    for file_name in _COPIED_FILES:
        shutil.copyfile(source_dir / file_name, recipe_dir / file_name)
    for file_name in _ROOT_FILES:
        if not (out_root / file_name).exists():
            shutil.copyfile(source_root / file_name, out_root / file_name)

    return LinearizeResult(
        recipe_dir=recipe_dir,
        trim=trim,
        model=model,
        coefficients=coefficients,
        dropped=dropped_terms(model),
    )


def print_result(result: LinearizeResult, file=sys.stdout) -> None:
    """Print the trim point, the coefficients and the dropped coupling terms."""
    trim = result.trim
    phi, theta, psi = _euler(trim)
    print(f"Wrote {result.recipe_dir}", file=file)
    print("\nTrim point", file=file)
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
            f"{label} {float(u):.6f}" for label, u in zip(_COLUMNS["B"], trim.controls)
        ),
        file=file,
    )
    print(
        f"  residual      {trim.residual:.3e} after {trim.evaluations} evaluations",
        file=file,
    )
    print("\nCoefficients", file=file)
    c = result.coefficients
    groups = (
        ("roll", ("Yv", "Ylat", "Lv", "Lp", "Llat")),
        ("pitch", ("Xu", "Xlon", "Mu", "Mq", "Mlon")),
        ("yaw", ("Nr", "Nped")),
        ("vertical", ("Zw", "Zcoll", "u_hover")),
    )
    for axis, keys in groups:
        values = ", ".join(f"{key} {getattr(c, key):.6g}" for key in keys)
        print(f"  {axis:<9} {values}", file=file)
    print("\nCoupling terms the recipe drops", file=file)
    if not result.dropped:
        print("  none", file=file)
    for term in result.dropped:
        print(f"  {term.matrix:<6} {term.label():<22} {term.value: .6g}", file=file)


def main() -> int:
    """Entry point of the `last-letter-linearize` console script."""
    parser = argparse.ArgumentParser(
        prog="last-letter-linearize",
        description=(
            "Trim a multicopter recipe at hover, linearize it and write a "
            "recipe with one LinearCopter body."
        ),
    )
    parser.add_argument(
        "recipe", help="The source recipe, for example ardupilot_copter."
    )
    parser.add_argument(
        "--name",
        help="The name of the output recipe. The default is <recipe>_linear.",
    )
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
        help="The models root of the source recipe. The default is the models path.",
    )
    parser.add_argument(
        "--out",
        type=Path,
        help=(
            "The models root to write into. The recipe goes to "
            "<out>/aircraft/<name>/. The default is the models path."
        ),
    )
    args = parser.parse_args()

    try:
        result = linearize_recipe(
            args.recipe, args.models_root, args.mixer, args.name, args.out
        )
    except (RuntimeError, FileNotFoundError) as error:
        print(error, file=sys.stderr)
        return 1
    print_result(result)
    return 0


if __name__ == "__main__":
    sys.exit(main())
