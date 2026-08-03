===============================================================================
Developers
===============================================================================

This page contains guidelines for *last_letter* developers.


Package purpose
===============================================================================

The *last_letter* package is meant to be a definitive data storage for aircraft recipes.
It is not meant to host simulation models.

However, since the model parameters are tightly dependent on the underlying mathematical
models, it is desirable to provide ample documentation when submitting a new model
type and its corresponding attributes.


Documentation
===============================================================================

Use docstrings to document every new class and **all** of its attributes. Use
full sentences with a fullstop at the end.

Testing
===============================================================================

Testing the Python library
--------------------------

Testing scripts are provided in the ``./lib/tests/`` directory.
The python package "pytest" is used for this purpose.

``nox`` is used as the testing framework. You can evoke all tests with ``nox``.

Testing the C++ library
-----------------------

This is not supported yet, as the tests build system used to depend on ROS 2.


Before submitting a Pull Request
===============================================================================

The CI actions will execute all of the tests upon every Pull Request. However,
this takes some time. To catch potential errors quicker, run ``nox`` locally
before creating the Pull Requests to execute the same tests locally faster.
