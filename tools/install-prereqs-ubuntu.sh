#!/usr/bin/env bash
#
# Installs the prerequisites for building last_letter on Ubuntu.
#
#   ./tools/install-prereqs-ubuntu.sh                  # C++ library and frontends
#   ./tools/install-prereqs-ubuntu.sh --python         # ... plus the Python package
#   ./tools/install-prereqs-ubuntu.sh --python --dev   # ... plus the test tooling
#   ./tools/install-prereqs-ubuntu.sh --python-only    # Python package, no C++ toolchain
#
# Installing Python prerequisites involves using a virtualenv. You are
# responsible for sourcing it manually every time or adding it to your shell profile.
set -euo pipefail

WANT_PYTHON=false
WANT_DEV=false
WANT_CPP=true
USE_UV=false
CMAKE_SOURCE=auto
APT_YES=""
INSTALL_HINT=""

usage() {
    # Reprint the header comment, stopping at the first line of actual code.
    awk 'NR == 1 { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "$0"
    cat <<'EOF'

Options:
  --python        Also set up the Python package (virtualenv + dependencies).
  --python-only   Skip the system cmake. The compiler and the C and C++ libraries
                  are still installed -- the Python package compiles them too --
                  but scikit-build-core fetches its own cmake, so a user who only
                  ever runs pip does not need one. Implies --python.
  --dev           Set up for development: the virtualenv goes in the repository
                  root as .venv, the install line printed at the end brings in
                  the development tooling (pytest, pre-commit), and the closing
                  pointer is to DEVELOPERS.md. Implies --python.
  --uv            Use uv for the virtualenv and the installs instead of
                  python3-venv and pip. uv must already be on PATH.
  --cmake=SOURCE  Where cmake comes from: auto (default), apt, kitware, pip, none.
  -y, --yes       Do not prompt during apt operations.
  -h, --help      Show this message.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --python)      WANT_PYTHON=true ;;
        --python-only) WANT_PYTHON=true; WANT_CPP=false ;;
        --dev)         WANT_PYTHON=true; WANT_DEV=true ;;
        --uv)          USE_UV=true ;;
        --cmake=*)     CMAKE_SOURCE="${1#--cmake=}" ;;
        -y|--yes)      APT_YES="-y" ;;
        -h|--help)     usage; exit 0 ;;
        *)             echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
    esac
    shift
done

case "$CMAKE_SOURCE" in
    auto|apt|kitware|pip|none) ;;
    *) echo "Unknown --cmake source: $CMAKE_SOURCE" >&2; exit 1 ;;
esac

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ---------------------------------------------------------------- distribution

if [ ! -r /etc/os-release ]; then
    echo "Cannot read /etc/os-release; this script only supports Ubuntu." >&2
    exit 1
fi
# shellcheck disable=SC1091
. /etc/os-release

if [ "${ID:-}" != "ubuntu" ]; then
    echo "This script targets Ubuntu; found ID=${ID:-unknown}." >&2
    echo "Debian is close enough to adapt by hand -- see README.md." >&2
    exit 1
fi

CODENAME="${VERSION_CODENAME:-}"
case "$CODENAME" in
    jammy|noble|resolute)
        echo "Ubuntu ${VERSION_ID} (${CODENAME}) is supported."
        ;;
    *)
        echo "Ubuntu ${VERSION_ID:-?} (${CODENAME:-?}) is untested; continuing anyway." >&2
        ;;
esac

SUDO=""
if [ "$(id -u)" -ne 0 ]; then
    SUDO="sudo"
fi
APT_GET="$SUDO apt-get $APT_YES"

$APT_GET update

# ------------------------------------------------------------------ C++ layer

# These are needed whichever half you are after. The Python package is not a
# pure-Python one -- installing it compiles the same C++ library from source --
# so there is no mode in which the compiler and these headers are optional.
BASE_PKGS="build-essential ca-certificates git libboost-dev libeigen3-dev liblz4-dev libzstd-dev"

# shellcheck disable=SC2086
$APT_GET install --no-install-recommends $BASE_PKGS

# The top-level CMakeLists.txt needs 3.28 for FetchContent_Declare's
# EXCLUDE_FROM_ALL keyword, which is newer than the cmake in Ubuntu 22.04.
CMAKE_FLOOR="3.28"

resolve_cmake_source() {
    local candidate
    candidate="$(apt-cache policy cmake 2>/dev/null | awk '/Candidate:/{print $2}')"
    if [ -n "$candidate" ] && dpkg --compare-versions "$candidate" ge "$CMAKE_FLOOR"; then
        echo apt
    else
        # Kitware rather than pip, because a C++ consumer has no reason to have
        # a Python toolchain at all.
        echo kitware
    fi
}

install_cmake_kitware() {
    echo "Adding the Kitware APT repository for cmake >= ${CMAKE_FLOOR}."
    # shellcheck disable=SC2086
    $APT_GET install --no-install-recommends ca-certificates gpg wget
    wget -qO- https://apt.kitware.com/keys/kitware-archive-latest.asc \
        | gpg --dearmor \
        | $SUDO tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
    echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ ${CODENAME} main" \
        | $SUDO tee /etc/apt/sources.list.d/kitware.list >/dev/null
    $APT_GET update
    # shellcheck disable=SC2086
    $APT_GET install --no-install-recommends cmake
}

# Only for callers that will run cmake themselves. --python-only skips this:
# scikit-build-core declares its own cmake requirement and pulls a suitable one
# from PyPI when the system is short, so a pip-only user never needs one here.
if [ "$WANT_CPP" = true ] && [ "$CMAKE_SOURCE" != none ]; then
    if [ "$CMAKE_SOURCE" = auto ]; then
        CMAKE_SOURCE="$(resolve_cmake_source)"
    fi
    case "$CMAKE_SOURCE" in
        apt)
            # shellcheck disable=SC2086
            $APT_GET install --no-install-recommends cmake
            ;;
        kitware)
            install_cmake_kitware
            ;;
        pip)
            echo "cmake will be installed into the virtualenv further down."
            ;;
    esac
fi

# --------------------------------------------------------------- Python layer

VENV_PATH=""
ACTIVATE_HINT=""

if [ "$WANT_PYTHON" = true ]; then
    if [ "$USE_UV" = true ]; then
        if ! command -v uv >/dev/null 2>&1; then
            echo "--uv was given but uv is not on PATH." >&2
            echo "Install it with:  curl -LsSf https://astral.sh/uv/install.sh | sh" >&2
            exit 1
        fi
        # shellcheck disable=SC2086
        $APT_GET install --no-install-recommends python3-dev
    else
        # shellcheck disable=SC2086
        $APT_GET install --no-install-recommends python3 python3-dev python3-pip python3-venv
    fi

    # An already-active virtualenv wins.
    # A tentative default.
    USER_VENV="${HOME}/venv-last-letter"

    if [ -n "${VIRTUAL_ENV:-}" ]; then
        VENV_PATH="$VIRTUAL_ENV"
        echo "Using the active virtualenv at ${VENV_PATH}."
    elif [ -d "${REPO_ROOT}/.venv" ]; then
        VENV_PATH="${REPO_ROOT}/.venv"
        echo "Using the existing virtualenv at ${VENV_PATH}."
    elif [ "$WANT_DEV" = false ] && [ -d "$USER_VENV" ]; then
        # Running the script a second time must find this one rather than remake
        # it: uv venv rebuilds an existing environment from scratch, which would
        # throw away whatever else the user had installed into it.
        #
        # Not for --dev, which stays on the in-repo .venv: that is the path
        # `uv sync` manages, and pointing a development install somewhere else
        # only creates two environments holding two copies of the extension.
        VENV_PATH="$USER_VENV"
        echo "Using the existing virtualenv at ${VENV_PATH}."
    else
        if [ "$WANT_DEV" = true ]; then
            VENV_PATH="${REPO_ROOT}/.venv"
        else
            VENV_PATH="$USER_VENV"
        fi
        echo "Creating a virtualenv at ${VENV_PATH}."
        # Deliberately not --system-site-packages, apt generally has old,
        # unsuported versions of our Python dependencies.
        if [ "$USE_UV" = true ]; then
            uv venv "$VENV_PATH"
        else
            python3 -m venv "$VENV_PATH"
        fi
    fi
    ACTIVATE_HINT="source ${VENV_PATH}/bin/activate"

    export VIRTUAL_ENV="$VENV_PATH"
    PATH="${VENV_PATH}/bin:${PATH}"
    export PATH

    if [ "$USE_UV" = false ]; then
        # 22.04 ships pip 22.0.2 and 24.04 ships 24.0; --group is PEP 735 and
        # needs 25.1. Upgrading inside the virtualenv costs nothing.
        pip install --upgrade pip
    fi

    # Pick the right Python package install command the user has to run.
    if [ "$USE_UV" = true ]; then
        if [ "$WANT_DEV" = true ]; then
            # Resolves from uv.lock and installs the project editable.
            if [ "$VENV_PATH" = "${REPO_ROOT}/.venv" ]; then
                INSTALL_HINT="uv sync"
            else
                # uv sync manages <project>/.venv and ignores VIRTUAL_ENV -- it
                # warns and carries on into .venv. Without --active the user
                # would activate the virtualenv named just above and then fill a
                # different one, and pytest would fail on the empty one.
                INSTALL_HINT="uv sync --active"
            fi
        else
            INSTALL_HINT="uv pip install ."
        fi
    else
        if [ "$WANT_DEV" = true ]; then
            INSTALL_HINT="pip install --group dev --editable ."
        else
            INSTALL_HINT="pip install ."
        fi
    fi
fi

# ------------------------------------------------------------------- next steps

cat <<EOF

Prerequisites installed.

EOF

if [ -n "$ACTIVATE_HINT" ]; then
    cat <<EOF
Activate the virtualenv, then build and install the Python package:

    ${ACTIVATE_HINT}
    ${INSTALL_HINT}

The second command compiles the C++ library as well as the bindings, so expect
it to take a few minutes. You might or might not want to add the first line to
your ~/.bashrc (or equivalent, depending on your shell).

Then put the aircraft models where the C++ library looks for them:

    last-letter-install-models

EOF
fi

# Point to C++ workflows.
if [ "$WANT_DEV" = true ]; then
    cat <<'EOF'
Building, testing and contributing to last_letter: see DEVELOPERS.md.

EOF
else
    cat <<'EOF'
Building and using last_letter: see README.md.

EOF
fi
