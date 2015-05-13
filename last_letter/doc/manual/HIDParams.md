##Human Interface Device (HID) Parameters

This file contains the mapping, scaling and inverting parameters needed to configure your input device. For more information on the input device configuraiton, please read the [corresponding guide](RCCal.md).

`axes`**(integer)**: The mapping between the input device axes and the PWM channels.

`buttons`**(integer)**: The mapping between the input device buttons and the PWM channels.

`throws`: The maximum value that the input device records at that channel. Is used for input scaling. If negative, the direction of control is inverted (similar to inverting a channel in a real-world transmitter).

[back to table of contents](../../../README.md)