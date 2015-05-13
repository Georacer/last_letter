##World Parameters

This file contains global parameters which control how the simulation behaves.

`simRate`: The simulation frame-rate (in Hz). This is the frame-rate which the simulator will try to achieve and which the user experiences. In other words, this is how fast the computer will try to perform the set of calculations corresponding to one simulation step.
**Note**: Depending on the initialization options, this parameter may not be used.

`deltaT`: The simulation time-step (in seconds). This is the increment of time which will be used for propagating the differential equations of the physics model. In other words, this is how much time will advance between each simulation step/frame **in the simulated world**.

`timeControls`: Index which selects the simulation frame-rate. Is used by the simulation clock node (`last_letter/src/clock.cpp`)
 * 0: The default value. The simulation calculates a new iteration at the rate defined by `simRate`.
 * 1: The simulation calculates a new iteration each time the `ctrlPWM` topic is updated. This allows for controllers which cannot work in real-time and need a lot of time to calculate the control input. It is also used to interface with `ardupilot` SITL.
 * 2: Free spinning clock, the simulation will update as fast as it can.

`integratorType`: Index which selects the type of integrator used for kinematics calculations. Right now only the *forward Euler* integrator is supported, with a value of 0.

[back to table of contents](../../../README.md)