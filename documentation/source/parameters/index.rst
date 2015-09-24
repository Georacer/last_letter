======================
last_letter parameters
======================

Most of `last_letter`'s libraries and executables are initialized (or operate based) upon several parameters. These parameters are loaded onto the *parameter server* on simulation startup by the launch file. The order in which the parameters are declared in the corresponding `*.yaml` file is not important, but declaring the same parameter twice in separate locations/files will overwrite the previous value.

.. warning::

	ll numerical values of the parameters within lists (in [] brackets) must be forced to a float format by a trailling ``.0`` if needed, unless otherwise stated

The main categories of parameters are the following:

.. toctree::
	:maxdepth: 2

	aircraftParams
	HIDParams
	environmentParams
	worldParams
	polynomialParams
