"""The last_letter_lib package"""

from . import aerodynamics
from . import environment
from . import gravity

# from . import model_creator
from . import propulsion

from . import simulation
from . import systems
from .utils import math
from .utils import programming
from .utils import uav

# Data-logging on/off toggle (all logging is driven from C++; Python only
# turns it on/off). Bound at the top level of the extension module.
from .cpp_last_letter_lib import enable_logging
from .cpp_last_letter_lib import disable_logging


__all__ = [
    "gravity",
    "environment",
    "systems",
    "aerodynamics",
    "propulsion",
    "simulation",
    # "model_creator",
    "programming",
    "math",
    "uav",
    "enable_logging",
    "disable_logging",
]
