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

CMake 3.28 or newer is required. This is newer than the 3.22 shipped by Ubuntu
22.04; install a current one with `uv tool install cmake` or from the
[Kitware APT repository](https://apt.kitware.com/).

The following system packages are also expected. On Debian/Ubuntu:

```bash
sudo apt install build-essential git libeigen3-dev libboost-dev liblz4-dev libzstd-dev
```

- `build-essential` — a C++17 compiler.
- `git` — the remaining dependencies (yaml-cpp, DataTamer and, for the tests,
  GoogleTest) are cloned at configure time by CMake's `FetchContent`, so the
  first configuration of a build directory needs network access.
- `libeigen3-dev` — Eigen 3, the linear algebra library. Located with
  `find_package(Eigen3)`.
- `libboost-dev` — only the header-only parts of Boost are used (`boost/array.hpp`
  and Boost.Odeint), so the compiled Boost libraries are not needed.
- `liblz4-dev`, `libzstd-dev` — the compression libraries used by mcap, which is
  vendored inside DataTamer. Without them the configure step fails in mcap's
  `FindZSTD.cmake`.

Building the Python bindings additionally needs the Python development headers,
`python3-dev`. `pybind11` itself is provided by the build environment and does
not have to be installed system-wide.

### To build and install the C++ code

```bash
cmake -S . -B build && cmake --build build && cmake --install build
```

Installation defaults to the `$HOME/.local` prefix, so no `sudo` is needed.
The library will be installed in `$HOME/.local/lib`.
Override it with `-DCMAKE_INSTALL_PREFIX=<path>`. Note that the aircraft models
are always installed to `$HOME/last_letter_models`, independently of the prefix.

A nix-shell pure configuration has been provided which can be optionally used.

To build the ArduPilot bridge binary run

```bash
cmake -S . -B build -DLAST_LETTER_BUILD_ARDUPILOT=ON && cmake --build build && cmake --install build
```

The executable will be installed in `$HOME/.local/bin`. Ensure it is on your path.

### To use the C++ library from another CMake project

The install exports a CMake package, so a downstream project only needs

```cmake
find_package(last_letter_lib REQUIRED)
target_link_libraries(myapp PRIVATE last_letter_lib::last_letter_lib)
```

That imported target carries the include directory and the Eigen and Boost
requirements, so no paths need hardcoding. If the library was installed to a
prefix CMake does not search by default, point at it with
`-DCMAKE_PREFIX_PATH=<prefix>`.

### To build the Python code

`last_letter` is distributed as a **source distribution** only — there is no
prebuilt wheel to download. Installing it compiles the C++ library on your own
machine, so the prerequisites above must be in place first, and the install
needs network access because CMake clones yaml-cpp and DataTamer while it
configures. CMake itself does not have to be installed: the build fetches a
suitable version if the system one is too old.

To build and install the Python package in your current Python environment enter:

```bash
uv pip install <path_to_last_letter> [--force-reinstall]
```

### The aircraft models and the Python package

A wheel carries the aircraft models inside the package, and

```python
from last_letter.paths import models_path
```

returns wherever they can be found — `$HOME/last_letter_models` if it exists,
the packaged copy otherwise. The C++ library only knows the first of those, so
after installing from a wheel, run

```bash
last-letter-install-models [--force]
```

to copy them out to `$HOME/last_letter_models`. The copy overlays the
destination instead of replacing it, so aircraft of your own kept there are left
alone.

## Developers

If you wish to modify and hopefully contribute to last_letter, please read
DEVELOPERS.md.
