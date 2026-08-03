#!/usr/bin/env python3
"""Library for environmental models

This library is meant to host utilities related to environmental models.

usage:
    Import the module for use in other scripts.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__date__ = "Thu 20 2022"
__copyright__ = "Copyright 2022, George Zogopoulos"

from abc import ABC
from abc import abstractmethod
from dataclasses import dataclass

from last_letter_lib.utils.math import Vector3

from .cpp_last_letter_lib.cpp_environment import EnvironmentData
from .cpp_last_letter_lib.cpp_environment import EnvironmentModel
