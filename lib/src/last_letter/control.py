#!/usr/bin/env python3
"""Trimmer and linearizer

The trimmer finds the trim point of an aircraft. The linearizer takes the
derivatives of its accelerations about that point.

usage:
    Import the module for use in other scripts.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__copyright__ = "Copyright 2026, George Zogopoulos"

from .cpp_last_letter.cpp_control import ControlMap
from .cpp_last_letter.cpp_control import CopterControlMap
from .cpp_last_letter.cpp_control import IdentityControlMap
from .cpp_last_letter.cpp_control import LinearCopterCoefficients
from .cpp_last_letter.cpp_control import LinearModel
from .cpp_last_letter.cpp_control import Linearizer
from .cpp_last_letter.cpp_control import LinearizerOptions
from .cpp_last_letter.cpp_control import RecipeTrim
from .cpp_last_letter.cpp_control import SettleOptions
from .cpp_last_letter.cpp_control import TrimOptions
from .cpp_last_letter.cpp_control import TrimPoint
from .cpp_last_letter.cpp_control import TrimTarget
from .cpp_last_letter.cpp_control import Trimmer
from .cpp_last_letter.cpp_control import build_mixer
from .cpp_last_letter.cpp_control import trim_recipe
from .cpp_last_letter.cpp_programming_utils import load_model_config

__all__ = [
    "ControlMap",
    "CopterControlMap",
    "IdentityControlMap",
    "LinearCopterCoefficients",
    "LinearModel",
    "Linearizer",
    "LinearizerOptions",
    "RecipeTrim",
    "SettleOptions",
    "TrimOptions",
    "TrimPoint",
    "TrimTarget",
    "Trimmer",
    "build_mixer",
    "load_model_config",
    "trim_recipe",
]
