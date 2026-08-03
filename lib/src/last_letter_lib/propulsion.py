#!/usr/bin/env python3
"""Propulsion-related libraries

Contains models and functions relevant to propulsion and thrusters.

usage:
    Import the module for use in other scripts.
"""

__authors__ = ["George Zogopoulos", "Johanna Windh"]
__credits__ = []
__date__ = "Fri 10 Sep 2021"
__copyright__ = "Copyright 2021, George Zogopoulos"

from .cpp_last_letter_lib.cpp_propulsion import Thruster
from .cpp_last_letter_lib.cpp_propulsion import NoEngine
from .cpp_last_letter_lib.cpp_propulsion import ThrusterSimple
from .cpp_last_letter_lib.cpp_propulsion import EngBeard
from .cpp_last_letter_lib.cpp_propulsion import ElectricEng
from .cpp_last_letter_lib.cpp_propulsion import ElectricEng2
from .cpp_last_letter_lib.cpp_propulsion import EngOmegaControl
from .cpp_last_letter_lib.cpp_propulsion import PistonEng

from .cpp_last_letter_lib.cpp_propulsion import Propeller
from .cpp_last_letter_lib.cpp_propulsion import PropellerStandard
