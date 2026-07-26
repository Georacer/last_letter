# last_letter_lib

Software libraries for UAV simulation.

## Build instructions

### Prerequisites

CMake 3.28 or newer is required. This is newer than the 3.22 shipped by Ubuntu
22.04; install a current one with `uv tool install cmake` or from the
[Kitware APT repository](https://apt.kitware.com/).

### To build and install the C++ code

```bash
cmake -S . -B build && cmake --build build && cmake --install build
```

Installation defaults to the `$HOME/.local` prefix, so no `sudo` is needed.
Override it with `-DCMAKE_INSTALL_PREFIX=<path>`. Note that the aircraft models
are always installed to `$HOME/last_letter_models`, independently of the prefix,
because that path is hardcoded in the library.

A nix-shell pure configuration has been provided which can be optionally used.

### To build the Python code

To build the Python wheels do

- uv build

To build the code so that you can import the Python code in editable mode do

- uv pip install --editable . [--force-reinstall]

## Testing instructions

### Build and run C++ tests

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DLAST_LETTER_BUILD_TESTS=ON && cmake --build build && ./build/all_tests
```

### Build everything

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DLAST_LETTER_BUILD_TESTS=ON -DLAST_LETTER_BUILD_PYTHON=ON && cmake --build build && cmake --install build && ./build/all_tests
```

### Run the Python tests

- uv run pytest tests/python
