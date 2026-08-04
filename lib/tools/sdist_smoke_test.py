#!/usr/bin/env python3
"""Check that a Python-installed last_letter is usable.

Written for the container in this directory, where it runs against a package
built from the source distribution.
"""

import subprocess
import sys
from pathlib import Path

import yaml

import last_letter
from last_letter import simulation as sim
from last_letter.paths import models_path

# One model file of each kind loadModelConfig() expects: the two at the root of
# the tree and one belonging to an individual aircraft.
EXPECTED_MODEL_FILES = (
    "world.yaml",
    "environment.yaml",
    "aircraft/skywalker_2013/aerodynamics.yaml",
)


def check(description: str, condition: bool) -> None:
    """Report a check and abort on failure."""
    print(f"{'ok  ' if condition else 'FAIL'} {description}")
    if not condition:
        sys.exit(1)


def main() -> None:
    package_dir = Path(last_letter.__path__[0])
    print(f"last_letter imported from {package_dir}")

    # Importing at all already proves the pybind extension loaded, and with it
    # that its $ORIGIN rpath found liblast_letter_lib.so next to it. Building
    # an object proves the C++ side is callable rather than merely present.
    aircraft = sim.Aircraft("smoke_test")
    check("the bound C++ Aircraft can be constructed", aircraft is not None)

    # Before the console script is run there is no $HOME copy, so this has to
    # resolve to the one inside the package.
    before = models_path()
    check(
        f"models_path() falls back to the packaged copy ({before})",
        before == package_dir / "models",
    )

    for relative_path in EXPECTED_MODEL_FILES:
        model_file = before / relative_path
        check(
            f"{relative_path} shipped and parses as YAML",
            model_file.is_file() and yaml.safe_load(model_file.read_text()) is not None,
        )

    # The models have to reach $HOME for anything driving the library from C++,
    # which knows no other location.
    installed = subprocess.run(["last-letter-install-models"], check=False)
    check("last-letter-install-models succeeds", installed.returncode == 0)
    check(
        f"models_path() now prefers {Path.home() / 'last_letter_models'}",
        models_path() == Path.home() / "last_letter_models",
    )

    # Rerunning must not silently overwrite a tree the user may have edited.
    rerun = subprocess.run(["last-letter-install-models"], check=False)
    check("a second run refuses without --force", rerun.returncode != 0)
    forced = subprocess.run(["last-letter-install-models", "--force"], check=False)
    check("a second run succeeds with --force", forced.returncode == 0)

    print("\nAll checks passed.")


if __name__ == "__main__":
    main()
