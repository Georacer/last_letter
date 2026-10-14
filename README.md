# last_letter

Software libraries for UAV simulation.

## Repository layout

```
lib/          the simulation library: C++ core, its Python package and the tests
lib/tools/    standalone analysis scripts and the packaging test container
frontends/    programs that drive the library, currently the ArduPilot SITL bridge
models/       aircraft recipes, installed to $HOME/last_letter_models
docs/         Sphinx documentation
```

The C++ library is called `last_letter_lib`, which is also the name of its CMake
target and of `liblast_letter_lib.so`; the suffix separates it from the frontends
that link it. The Python package is simply `last_letter`.

## Build instructions

### Prerequisites

Supported OS: Ubuntu 22.04, 24.04, 26.04.

A script is provided that installs all the prerequisites you need before
building and using last_letter:

```bash
./tools/install-prereqs-ubuntu.sh                  # C++ library and frontends
./tools/install-prereqs-ubuntu.sh --python         # ... plus the Python package
./tools/install-prereqs-ubuntu.sh --python --dev   # ... plus pytest and pre-commit
./tools/install-prereqs-ubuntu.sh --python-only    # Python package, no C++ toolchain
```

Add `--uv` to fill the virtualenv with `uv` instead of `pip`. The script never
installs into the system Python interpreter and never edits your shell profile. It
prints the activation line for any virtualenv it creates, and reuses one that is
already active rather than making a second.

You will get recommendations on how to build and install last_letter, depending
on your options.

### To build and install the C++ library

```bash
cmake -S . -B build && cmake --build build && cmake --install build
```

Installation defaults to the `$HOME/.local` prefix, so no `sudo` is needed.
The library will be installed in `$HOME/.local/lib`.
Override it with `-DCMAKE_INSTALL_PREFIX=<path>`. Note that the aircraft models
are always installed to `$HOME/last_letter_models`, independently of the prefix.

### To build and install the ArduPilot bridge

```bash
cmake -S . -B build -DLAST_LETTER_BUILD_ARDUPILOT=ON && cmake --build build && cmake --install build
```

The executable will be installed in `$HOME/.local/bin`. Ensure it is on your path.

The models root it reads defaults to `$HOME/last_letter_models`, and is overridden
either with `--models-dir <path>` or with the `LAST_LETTER_MODELS_DIR` environment
variable.

Prebuilt binaries are provided in the [Githug releases](https://github.com/Georacer/last_letter/releases) tarballs.

### To use the C++ library from another CMake project

The install exports a CMake package, so a downstream project only needs

```cmake
find_package(last_letter_lib REQUIRED)
target_link_libraries(myapp PRIVATE last_letter_lib::last_letter_lib)
```

If the library was installed to a prefix CMake does not search by default,
point at it with `-DCMAKE_PREFIX_PATH=<prefix>`.

### To build the Python code

`last_letter` is distributed as a **source distribution** only — there is no
prebuilt wheel to download. Installing it compiles the C++ library on your own
machine, so the prerequisites above must be in place first.

To build and install the Python package in your current Python environment enter:

```bash
uv pip install <path_to_last_letter> [--force-reinstall]
```

or, without `uv`, from inside an activated virtualenv:

```bash
pip install <path_to_last_letter> [--force-reinstall]
```

### The aircraft models and the Python package

After installing the Python package, run

```bash
last-letter-install-models [--force]
```

to copy the models out to `$HOME/last_letter_models`. The copy overlays the
destination instead of replacing it, so aircraft of your own kept there are left
alone.

Two recipes are made to fly with the ArduPilot bridge. `ardupilot_plane` is the
default aircraft for ArduPlane, and `ardupilot_copter` is a copy of the default
ArduCopter SITL quad frame and the default aircraft for ArduCopter. Fly them with

```bash
sim_vehicle.py -v ArduPlane --model last_letter:ardupilot_plane
sim_vehicle.py -v ArduCopter --model last_letter:ardupilot_copter
```

## Developers

If you wish to modify and hopefully contribute to last_letter, please read
DEVELOPERS.md.
