## Rigid Body Parameters

These are found in the file `rigidBody.yaml` and assign values to the rigid-body quantities of the aircraft.

`m` Mass of the aircraft (in kg).

`CGOffset` The location of the center of lift in terms of the center of gravity, in X/Y/Z body frame coordinates. Negative X means that the CoL is behind the CoG. Positive Y means that the CoL is to the right of the CoG. Negative Z means that the CoL is above the CoG.

`J_*` The element of the matrix of inertia of the aircraft. So far, only `J_x`, `J_y`, `J_z` and `J_xz` are supported, but the intention is to add support for a full inertia matrix.

[back to table of contents](../../../README.md)