# Runs one cell of the Ubuntu test matrix. A successful `docker build` is the
# pass condition; nothing has to be run afterwards.
#
# The matrix is (release) x (cell). Pick the release with BASE_IMAGE and the
# cell with --target, always from the repository root:
#
# Examples:
#   docker build -f lib/tools/Dockerfile --target cpp        --build-arg BASE_IMAGE=ubuntu:24.04 .
#   docker build -f lib/tools/Dockerfile --target consumer   --build-arg BASE_IMAGE=ubuntu:24.04 .
#   docker build -f lib/tools/Dockerfile --target python-pip --build-arg BASE_IMAGE=ubuntu:24.04 .
#   docker build -f lib/tools/Dockerfile --target python-uv  --build-arg BASE_IMAGE=ubuntu:24.04 .
#   docker build -f lib/tools/Dockerfile --target sdist      --build-arg BASE_IMAGE=ubuntu:24.04 .
#
# Supported releases are:
#   ubuntu:22.04
#   ubuntu:24.04
#   ubuntu:26.04
#
# The cells:
#
#   cpp         The C++ library, the ArduPilot bridge and the googletest suite,
#               built the way a C++ consumer builds them: apt only, no Python.
#               Use --target cpp-build to stop before running the tests.
#   consumer    A throwaway downstream project doing find_package(last_letter_lib)
#               against the installed CMake package.
#   python-pip  The Python package installed editable with the distribution's own
#               python3 and pip, then pytest.
#   python-uv   The same through uv.
#   sdist       A tarball, rolled from this tree, installed on a bare image, then
#               the smoke test. This is the release check and is the default
#               target. It needs no prior `uv build`.
#
# Every cell provisions through tools/install-prereqs-ubuntu.sh, the same script
# the README tells a user to run, so a green cell means the documented install
# path works on that release.

ARG BASE_IMAGE=ubuntu:22.04

# ---------------------------------------------------------------------- prereqs
# Only the script is copied here, so editing a source file does not invalidate
# the apt layer. The later stages re-run the script to add the Python pieces;
# by then apt has nothing left to fetch.
FROM ${BASE_IMAGE} AS prereqs
ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /repo
COPY tools/install-prereqs-ubuntu.sh tools/
RUN bash tools/install-prereqs-ubuntu.sh -y

FROM prereqs AS source
COPY . /repo

# ------------------------------------------------------------------ cpp-build
# Building and testing are separate stages on purpose. If `consumer` sat
# downstream of the test run, a single failing test would make the installed
# CMake package untestable.
FROM source AS cpp-build
RUN cmake -S /repo -B /build \
        -DLAST_LETTER_BUILD_TESTS=ON \
        -DLAST_LETTER_BUILD_ARDUPILOT=ON \
    && cmake --build /build -j"$(nproc)" \
    && cmake --install /build

# -------------------------------------------------------------------------- cpp
FROM cpp-build AS cpp
WORKDIR /build
RUN /build/all_tests

# --------------------------------------------------------------------- consumer
# Guards the installed surface, ensuring that the exported CMake package
# resolves, that its find_dependency calls are complete, and that the installed
# headers still compile against the bare minimum dependencies.
FROM cpp-build AS consumer
WORKDIR /consumer
RUN printf '%s\n' \
        'cmake_minimum_required(VERSION 3.28)' \
        'project(consumer LANGUAGES CXX)' \
        'find_package(last_letter_lib REQUIRED)' \
        'add_executable(consumer main.cpp)' \
        'target_link_libraries(consumer PRIVATE last_letter_lib::last_letter_lib)' \
        > CMakeLists.txt \
    && printf '%s\n' \
        '#include <cmath>' \
        '#include <last_letter_lib/geo_mag_declination.hpp>' \
        '#include <last_letter_lib/uav_model.hpp>' \
        '#include <last_letter_lib/control/linearize.hpp>' \
        'int main(int argc, char **)' \
        '{' \
        '    // Call an out-of-line symbol, so the link has to resolve against' \
        '    // liblast_letter_lib.so rather than merely name it. An empty main()' \
        '    // still records a DT_NEEDED today, but only because plain g++ does' \
        '    // not pass --as-needed; one LDFLAGS away, the whole check evaporates.' \
        '    // argc keeps the call out of reach of constant folding.' \
        '    const float declination = last_letter_lib::get_mag_declination(37.98F * argc, 23.73F);' \
        '    return std::isfinite(declination) ? 0 : 1;' \
        '}' \
        > main.cpp \
    && cmake -S /consumer -B /consumer/build -DCMAKE_PREFIX_PATH=/root/.local \
    && cmake --build /consumer/build
# Running it proves the installed library is loadable, not just linkable.
RUN /consumer/build/consumer

# ------------------------------------------------------------------- python-pip
FROM source AS python-pip
RUN bash tools/install-prereqs-ubuntu.sh --dev -y
RUN . /repo/.venv/bin/activate && pip install --group dev --editable .
RUN . /repo/.venv/bin/activate && pytest lib/tests/python

# -------------------------------------------------------------------- python-uv
FROM source AS python-uv
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv
# Pin uv to the distribution's interpreter. Left to itself it would honour
# .python-version and fetch the same managed Python on every release, which
# would quietly collapse the interpreter axis of the matrix.
ENV UV_PYTHON=python3
RUN bash tools/install-prereqs-ubuntu.sh --dev --uv -y
RUN uv sync
RUN . /repo/.venv/bin/activate && pytest lib/tests/python

# ------------------------------------------------------------------ sdist-build
# Build the sdist tarball.
FROM source AS sdist-build
RUN bash tools/install-prereqs-ubuntu.sh --python-only -y \
    && python3 -m venv /opt/sdistenv \
    && /opt/sdistenv/bin/pip install --no-cache-dir --upgrade pip build \
    && /opt/sdistenv/bin/python -m build --sdist --outdir /sdist /repo

# ------------------------------------------------------------------------ sdist
# Checks that the tarball installs on a machine that has only the tarball and the
# documented prerequisites.
#
# Last on purpose, so it stays the default target.
FROM ${BASE_IMAGE} AS sdist
ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /repo
COPY tools/install-prereqs-ubuntu.sh tools/
RUN bash tools/install-prereqs-ubuntu.sh --python-only -y

# The script puts a non-dev virtualenv here and installs into an active one, so
# activating it is all that is needed to make pip land in the right place.
ENV VIRTUAL_ENV=/root/venv-last-letter
ENV PATH=${VIRTUAL_ENV}/bin:${PATH}

COPY --from=sdist-build /sdist/*.tar.gz /tmp/

# The compile happens here, and takes a few minutes.
RUN pip install --no-cache-dir /tmp/*.tar.gz

COPY lib/tools/sdist_smoke_test.py /tmp/
RUN python /tmp/sdist_smoke_test.py

CMD ["python"]
