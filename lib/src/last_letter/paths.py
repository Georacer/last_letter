"""Locations of the data files the simulator reads at runtime."""

import argparse
import shutil
import sys
from pathlib import Path

# Shipped inside the wheel, staged there by the top-level CMakeLists.txt.
_PACKAGED_MODELS = Path(__file__).parent / "models"

# The default path that the C++ programming_utils::loadModelConfig() falls
# back to when handed an empty root.
_INSTALLED_MODELS = Path.home() / "last_letter_models"


def models_path() -> Path:
    """Return the root of the aircraft-model tree.

    Both copies can exist at once on a development machine. The one under $HOME
    wins, so that Python and C++ read the same models there - the C++ side has
    no way to discover the packaged copy, since it is reachable only through the
    importing interpreter. The packaged copy is the fallback that makes a plain
    `pip install last-letter` usable from Python without any further step.

    :raises FileNotFoundError: if neither location exists.
    """
    for candidate in (_INSTALLED_MODELS, _PACKAGED_MODELS):
        if candidate.is_dir():
            return candidate
    raise FileNotFoundError(
        f"No aircraft models found. Looked in {_INSTALLED_MODELS} (written by "
        f"`cmake --install` or `last-letter-install-models`) and "
        f"{_PACKAGED_MODELS} (shipped in the wheel)."
    )


def install_models(destination: Path = _INSTALLED_MODELS, force: bool = False) -> int:
    """Copy the models shipped in the package out to `destination`.

    Invoked manually to install the packaged models onto
    $HOME/last_letter_models, where the user can manipulate them.

    :param destination: Where to copy to.
    :param force: Overwrite an existing destination instead of refusing.
    :return: The number of files copied.
    :raises FileExistsError: if `destination` exists and `force` is false.
    :raises FileNotFoundError: if the package carries no models, which is the
        case for an editable install, where `cmake --install` is the way to
        populate the destination instead.
    """
    if not _PACKAGED_MODELS.is_dir():
        raise FileNotFoundError(
            f"This installation ships no models at {_PACKAGED_MODELS}; it is "
            "probably an editable install. Run `cmake --install <build dir>` "
            "instead, which writes them to the same place."
        )
    if destination.exists() and not force:
        raise FileExistsError(
            f"{destination} already exists. Pass force to overwrite the files "
            "it has in common with the packaged models."
        )

    shutil.copytree(_PACKAGED_MODELS, destination, dirs_exist_ok=True)
    return sum(1 for path in _PACKAGED_MODELS.rglob("*") if path.is_file())


def main() -> int:
    """Entry point of the `last-letter-install-models` console script."""
    parser = argparse.ArgumentParser(
        prog="last-letter-install-models",
        description=(
            "Copy the aircraft models shipped with the package to "
            f"{_INSTALLED_MODELS}, where the simulation library looks for them "
            "by default."
        ),
    )
    parser.add_argument(
        "-d",
        "--destination",
        type=Path,
        default=_INSTALLED_MODELS,
        help="Copy somewhere other than the default location.",
    )
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        help="Overwrite an existing destination.",
    )
    args = parser.parse_args()

    try:
        num_files = install_models(args.destination, args.force)
    except FileExistsError:
        print(
            f"{args.destination} already exists. Pass --force to overwrite the "
            "files it has in common with the packaged models; any others it "
            "holds are left alone either way.",
            file=sys.stderr,
        )
        return 1
    except FileNotFoundError as error:
        print(error, file=sys.stderr)
        return 1

    print(f"Copied {num_files} files -> {args.destination}")
    return 0
