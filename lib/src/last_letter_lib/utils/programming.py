#!/usr/bin/env python3
"""General utilities library

This library is meant to host miscellaneous functions offering general
programming functionality.

usage:
    Import the module for use in other scripts.
"""

__authors__ = ["George Zogopoulos", "Johanna Windh"]
__credits__ = []
__date__ = "Fri 10 Sep 2021"
__copyright__ = "Copyright 2021, George Zogopoulos"

import logging
from typing import List

import matplotlib.pyplot as plt

# from pkg_resources import resource_filename
from pydantic import BaseModel


def num(s):
    """convert string to number, interger or float
    INPUT: string
    OUTPUT: float or integer"""
    try:
        return int(s)
    except ValueError:
        return float(s)


def isfloat(string):
    """checks if string is integer or float
    INPUT: string
    OUTPUT: returns true if it is float and false if it's not float"""
    try:
        float(string)
        return True
    except ValueError:
        return False


# Probably needs removing
# def get_artifacts_path():
#     """Return the path to the artifacts directory. This directory is ignored by git."""
#     return resource_filename("last_letter_lib", "output_artifacts")


# Probably needs removing
# def get_data_path():
#     """Return the path to the data directory."""
#     return resource_filename(
#         "last_letter_lib", "data"
#     )  # Set name of the directory to access data.


def build_colorlist(num_plots, colormap="viridis"):
    """
    Return an iterator of colors for plotting num_plots different plots.
    """
    if num_plots <= 1:
        return ["b"]
    else:
        c_map = plt.cm.get_cmap(colormap)
        colorlist = [c_map(1.0 * i / (num_plots - 1) + 0.001) for i in range(num_plots)]
        return colorlist


def _traverse_collect_pydantic(
    inmodel: BaseModel, model_type: BaseModel
) -> List[BaseModel]:
    """
    Recursively traverse a pydantic object and collect all of the (sub) models of a certain type.

    Inspired from https://stackoverflow.com/questions/12507206/how-to-completely-traverse-a-complex-dictionary-of-unknown-depth

    :param inmodel: The pydantic model to traverse.
    :param model_type: The type of pydantic model to look for. Typically a subclass of BaseModel made by the User.
    :param pre: An auxiliary list, helping the recursion.
    :return: A list containing the desired objects.
    """

    # If we found our model type, return it
    if isinstance(inmodel, model_type):
        logging.debug(f"Found a {inmodel.__class__.__name__}, yielding.")
        yield inmodel
    # Otherwise continue parsing deeper
    elif isinstance(inmodel, BaseModel):
        logging.debug("Found a BaseModel, expanding.")
        for key, _ in inmodel.__fields__.items():
            value = getattr(inmodel, key)
            logging.debug(f"Found key {key} of type {type(value)}, examining.")
            # If this is any other pydantic model continue parsing
            if isinstance(value, BaseModel):
                logging.debug("Found a BaseModel value, recursion.")
                yield from _traverse_collect_pydantic(value, model_type)
            # If it's a list, continue parsing
            elif isinstance(value, list) or isinstance(value, tuple):
                logging.debug("Found a sequence value, recursion.")
                for v in value:
                    yield from _traverse_collect_pydantic(v, model_type)
            # Otherwise this is an unwanted value, skip
            else:
                continue
    else:
        # This is a simple-value field, do nothing
        logging.debug(f"Ignoring value of type {type(inmodel)}.")
        pass
