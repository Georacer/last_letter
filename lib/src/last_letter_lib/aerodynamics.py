#!/usr/bin/env python3
"""Aerodynamics-related libraries

Contains functions and classes that relate to aerodynamics calculations.

usage:
    Import the module for use in other scripts.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__date__ = "Fri 24 Dec 2022"
__copyright__ = "Copyright 2022, George Zogopoulos"


from .cpp_last_letter_lib.cpp_aerodynamics import calc_dynamic_pressure
from .cpp_last_letter_lib.cpp_aerodynamics import calc_bank_from_radius
from .cpp_last_letter_lib.cpp_aerodynamics import Aerodynamics
from .cpp_last_letter_lib.cpp_aerodynamics import NoAerodynamics
from .cpp_last_letter_lib.cpp_aerodynamics import StdLinearAero
from .cpp_last_letter_lib.cpp_aerodynamics import SimpleDrag
from .cpp_last_letter_lib.cpp_aerodynamics import PolynomialAero

import numpy as np

from scipy.optimize import minimize_scalar


def calc_max_l_d(Cl_coeffs, Cd_coeffs, xtol=0.0001):
    """
    Find the maximum lift over drag ratio and the corresponding AoA.

    Calculate the optimum angle of attack at which best lift over drag ratio is achieved.
    This coincides with the best range operational point.

    INPUTS:     Ct_coeffs: an iterator with the polynomial coefficients of the lift curve.
                    Higest degree first. Sized for radians.
                Cp_coeffs: an iterator with the polynomial coefficients of the drag curve.
                    Higest degree first. Sized for radians.
                xtol: the accuracy of the optimal AoA, in radians.
    OUTPUTS:    best_l_d: optimal lift/drag ratio
                best_aoa: optimal angle of attack angle
    """

    def cost_function(x):
        return -np.polyval(Cl_coeffs, x) / np.polyval(Cd_coeffs, x)

    method = "brent"
    options = dict()
    options["xtol"] = xtol
    glide_ratio_bounds = 30

    try:
        optim_res = minimize_scalar(cost_function, method=method, options=options)
    except (
        RuntimeWarning
    ):  # Ensure there is no number overflow warning, signifying failure
        raise RuntimeError("Errors occurred during optimization.")
    if abs(optim_res.fun) > glide_ratio_bounds:
        optim_res.success = False
    if not optim_res.success:
        raise RuntimeError("Could not find an optimal glide solution.")

    return (-optim_res.fun, optim_res.x)
