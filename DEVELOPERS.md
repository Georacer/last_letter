# Information for developers

## Building

It is recommended that you use `uv` to interact with the Python package.

### Building the Python package

To build the distributable archive, which is placed under `dist`:

```bash
uv build --sdist
```

### To check that the source distribution installs

`lib/tools/Dockerfile` builds a source distribution from the working tree,
installs it onto a clean system and exercises the result, so that a missing
prerequisite shows up here rather than in a user's terminal:

```bash
docker build -f lib/tools/Dockerfile -t last-letter-sdist .
```

A successful build is the pass condition; there is nothing to run afterwards.
The base image defaults to Ubuntu 22.04 and can be changed with
`--build-arg BASE_IMAGE=<image>`, subject to the `requires-python` bound in
`pyproject.toml`.

### To run the Ubuntu test matrix

The same Dockerfile carries the rest of the matrix. Choose the release with
`BASE_IMAGE` and the cell with `--target`:

```bash
docker build -f lib/tools/Dockerfile --target cpp        --build-arg BASE_IMAGE=ubuntu:24.04 .
docker build -f lib/tools/Dockerfile --target consumer   --build-arg BASE_IMAGE=ubuntu:24.04 .
docker build -f lib/tools/Dockerfile --target python-pip --build-arg BASE_IMAGE=ubuntu:24.04 .
docker build -f lib/tools/Dockerfile --target python-uv  --build-arg BASE_IMAGE=ubuntu:24.04 .
```

`cpp` builds the library, the ArduPilot bridge and the googletest suite from apt
packages alone, the way a C++ consumer does. `consumer` compiles a throwaway
downstream project against the installed CMake package. The two `python-*` cells
install the package editable and run pytest, through the distribution's own
`python3` and `pip` or through `uv`. Every cell, `sdist` included, provisions
with `tools/install-prereqs-ubuntu.sh`, so a green cell means the install path
the README documents actually works on that release.

Supported releases are `ubuntu:22.04`, `ubuntu:24.04` and `ubuntu:26.04`.

## Testing instructions

### Build and run C++ tests

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DLAST_LETTER_BUILD_TESTS=ON && cmake --build build && ./build/all_tests
```

The test fixtures are found through a path baked in at configure time, so
`all_tests` can be run from any working directory.

### Build everything

To compile everything, including the Python bindings, and run the GoogleTests for the C++ codebase:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug \
    -DLAST_LETTER_BUILD_TESTS=ON \
    -DLAST_LETTER_BUILD_PYTHON=ON \
    -DLAST_LETTER_BUILD_ARDUPILOT=ON \
    && cmake --build build \
    && ./build/all_tests
```

### Run the Python tests

First build and install the Python package (see above). Then enter:

```bash
uv run pytest lib/tests/python
```

### Fly a copter recipe in ArduPilot SITL

`lib/tools/sitl_hover_check.py` flies a recipe in ArduCopter SITL through the bridge. It takes
off in GUIDED to 10 m, holds for 20 s, lands and disarms. It exits 0 when the altitude stays within
1 m and the roll and pitch within 5 degrees during the hold. Install the bridge and the models from
a Release tree first, then run the check on the stock quad before your own recipe, so that a
failure points at the recipe and not at the harness:

```bash
cmake --build build-release && cmake --install build-release
uv run --with pymavlink lib/tools/sitl_hover_check.py ardupilot_copter
uv run --with pymavlink lib/tools/sitl_hover_check.py <recipe>
```

It expects an ArduPilot checkout with a built `arducopter`, which `--ardupilot` points at. On a
fail it prints the last `STATUSTEXT` lines and the paths of the ArduPilot logs.
