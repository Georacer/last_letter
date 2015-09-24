Human Interface Device (HID) Parameters
=======================================

This file contains the mapping, scaling and inverting parameters needed to configure your input device. All the following parameters must be prefixed under the ``/HID/`` group.
For more information on the input device configuraiton, please read the `corresponding guide <../getting_started/RCCal.html>`_.

``axes``**(integer)**: The mapping between the input device axes and the PWM channels.

``buttons``**(integer)**: The mapping between the input device buttons and the PWM channels.

``throws``: The maximum value that the input device records at that channel. Is used for input scaling. If negative, the direction of control is inverted (similar to inverting a channel in a real-world transmitter).

``mixerid``: The input mixing scheme. Currently, there are 3 types of mixers implemented:
- 0: No mixing - the aircraft model will receive the channels exactly as they are remapped and scaled by the calibration procedure.
- 1: Airplane mixing - The throttle channel will be re-scaled from [-1 1] to [0 1] to be used by a motor and channel 0 will be also be mapped from [-1 1] to [0 1], in order to suitably control the simulation reset function.
- 2: Quadrotor mixing - The first 4 channels are mixed with each other to produce meaningful motor signals for an X-frame quadrotor. A reset channel is also configured.
- 3: Firefly Y6 mixing - Mixing suitable for a somewhat clunky control scheme of the Firefly Y6 hybrid vehicle.
