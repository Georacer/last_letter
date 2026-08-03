#!/usr/bin/env python3
"""Library for gravity models

This library is meant to host utilities related to gravity models.

usage:
    Import the module for use in other scripts.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__date__ = "Thu 20 2022"
__copyright__ = "Copyright 2022, George Zogopoulos"


from .cpp_last_letter_lib.cpp_gravity import GravityClassic
from .cpp_last_letter_lib.cpp_gravity import GravitySimple
