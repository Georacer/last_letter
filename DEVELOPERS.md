# Information for developers

## Building

It is recommended that you use `uv` to interact with the Python package.

### Building the Python package

To build the distributable archive, which is placed under `dist`:

```bash
uv build --sdist
```

### To check that the source distribution installs

`lib/tools/Dockerfile` installs the archive from `dist` onto a clean system and
exercises the result, so that a missing prerequisite shows up here rather than
in a user's terminal:

```bash
uv build --sdist
docker build -f lib/tools/Dockerfile -t last-letter-sdist .
```

A successful build is the pass condition; there is nothing to run afterwards.
The base image defaults to Ubuntu 22.04 and can be changed with
`--build-arg BASE_IMAGE=<image>`, subject to the `requires-python` bound in
`pyproject.toml` — Ubuntu 24.04 and later ship Python 3.12, which the current
bound of `<3.12` excludes.

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
