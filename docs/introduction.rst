===============================================================================
Introduction
===============================================================================

.. *last_letter* is a Python package meant to serve two purposes:

.. 1. Be the *source of truth* for all aircarft in terms of:

..   * mass and mass distribution
..   * aerodynamic parameters
..   * powertrain
..   * and any other parameters which affect vehicle performance

.. 2. Provide a useful API, so that other tools can make use of this information.

.. The basis of each aircraft description is a set of .yaml files. Each .yaml file
.. is human readable and editable by any text editor. However, its structure is
.. rigid and fixed, to guarantee that the last_letter API will be stable and won't break
.. the functionality for its dependant tools.

.. The correct structure is enforced by the code itself, that doesn't accept the
.. .yaml files unless they have the correct structure.
.. Additionally, with each commit or PR to this repository, a CI job is run that
.. checks the aircraft data for validity.
