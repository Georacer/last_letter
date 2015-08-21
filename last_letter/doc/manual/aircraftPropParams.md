## Aircraft Propulsion Parameters

This file contains the parameters of the propulsion model.

The propulsion of aircraft in `last_letter` is modelled as the sum of separate thrusters, placed on arbitrary points on the airframe and in arbitrary orientations. In that manner, a single forward-facing motor can model an airplane power plant, just as easilly as four upwards-facing motors can be used to simulate a quadrotor.

`motor/nMotors`: The number of thrusters mounted on the aircraft.


**Note** The following parameters are referring to a single thruster. They are groupped under the prefix `motorX/`, where `X` is the number-id of the thruster, starting from 1 for a single-thruster aircraft (eg `motor1/CGOffset`)

`chanMotor`: The input channel which controls the thruster output. Set to -1 for no thruster control.

`CGOffset`: The location of the thrust application point in terms of the center of gravity, in X/Y/Z body frame coordinates. Negative X means that the application point is behind the CoG. Positive Y means that the point is to the right of the CoG. Negative Z means that the point is above the CoG.

`mountOrientation`: The rotation Roll-Pitch-Yaw (in this sequence) rotation angles which rotate the thruster body from the Body Frame to its actual [**Propeller Frame**](referenceFrames.md) fixture. Since thrust is mostly generated towards the positive-x direction of the **Propeller Frame**, for a simplified airplane motor installation on the CoG, this should be [0.0,0.0,0.0].
Care is needed at this point, in case this thruster is gimballed. `last_letter` gimbals thrusters around the **Propeller Frame** z-axis. Consequently, `mountOrientation` should be selected in such a way that gimballing will end-up occuring around the correct axis.

`chanGimbal`: The input channel which controls the thruster gimbal. Set to -1 for no gimballing action.

`gimbalAngle_max`: The maximum throw angle (in radians) that the gimbal is providing when given full input.

`rotationDir`: The direction at which the thruster propeller is rotating. This is used both for visualization, as well as for counter-torque calculation purposes. **Valid values are 1 or -1**.

There are 4 types of thruster models available. These are selected with the variable
`motorType`:
- 0: Selects dummy thruster model, which has no interactions with the surrounding air. Useful for debugging.
- 1: Beard Engine - Simplified electric motor - propeller model, as provided in the book `Small Unmanned Aircraft: Theory and Practice, Randal W. Beard & Timothy W. McLain`.
- 2: Internal combustion engine - This model combines an IC engine model with a propeller model. Torque curves are specified for the engine, as well as propeller power and efficiency curves.
- 3: Electric motor - This model combines an electric motor model with a propeller model. This configuration is common to many small-scale RC aircraft. A 3-parameter model is used for the electric motor and power and efficiency curves are used for the propeller modelling.

### No thruster model
No parameters need to be defined regarding this model.

### Beard thruster model
The thrust and torque models for this engine are
```C++
wrenchProp.force.x = 1.0/2.0*rho*s_prop*c_prop*(pow(inputMotor * k_motor,2)-pow(airspeed,2));
wrenchProp.torque.x = -rotationDir * k_t_p*pow(omega,2);
```
The related parameters are:
`s_prop`: The propeller disc area

`k_motor`: A thruster coefficient multiplying the control input, related to the thrust output

`k_t_p`: Coefficient of moment produced by the thruster, multiplied by the square of the angular velocity of the motor.

`k_omega`: Coefficient which multiplies the control input (which has a range of [0,1]) to produce the motor angular velocity.

`c_prop`: Parameter reflecting the thrust efficiency of the thruster. This can be set to 1.0 and have its effect merged in `s_prop`.

### Internal combustion engine
`engPowerPoly`: A 1-variable (1D) polynomial describing the engine horsepower (HP) as a function of the motor RPM, HP=f(RPM). This is a parameter group and all the related polynomial parameters must be set as a parameter under this group.
More information on the related parameters can be found in the [polynomial parameters page](polynomialParams.md).

`nCoeffPoly`: A 1-variable (1D) polynomial describing the propeller efficiency factor (non-dimensional) as a function of the *advance ratio*. This is a parameter group and all the related parameters must be set as a parameter under this group.
More information on the related parameters can be found in the [polynomial parameters page](polynomialParams.md).

`propPowerPoly`: A 1-variable (1D) polynomial describing the power coefficient of the propeller () as a function of the *advance ratio*. This is a parameter group and all the related parameters must be set as a parameter under this group.
More information on the related parameters can be found in the [polynomial parameters page](polynomialParams.md).

`RadPSLimits`: The engine radians-per-second lower and upper limit, provided in a list, eg `RadPSLimits: [314.16, 733.04]`.

`propDiam`: The propeller diameter in meters.

`engInertia`: The combined engine and propeller moment of inertia, in kg*m^3.

### Electric motor
#### Motor specifications
`Kv`: The well-known KV rating of electric motors, which is the number of revolutions per volt applied on the terminals, on a unloaded motor, in RPS/V.

`Rm`: The internal resistance of the motor, in Ohms.

`I0`: The activation current of the motor, Amperes

`RadPSLimits`: he engine radians-per-second lower and upper limit, provided in a list, eg `RadPSLimits: [0.01, 1000.0]`.

#### Propeller specifications
`propDiam`: The propeller diameter in meters.

`nCoeffPoly`: A 1-variable (1D) polynomial describing the propeller efficiency factor (non-dimensional) as a function of the *advance ratio*. This is a parameter group and all the related parameters must be set as a parameter under this group.
More information on the related parameters can be found in the [polynomial parameters page](polynomialParams.md).

`propPowerPoly`: A 1-variable (1D) polynomial describing the power coefficient of the propeller () as a function of the *advance ratio*. This is a parameter group and all the related parameters must be set as a parameter under this group.
More information on the related parameters can be found in the [polynomial parameters page](polynomialParams.md).

#### Other specifications
`engInertia`: The combined engine and propeller moment of inertia, in kg*m^3.

`Rs`: Battery internal resistance in Ohms.

`Cells`:(**integer**) Battery number of LiPo cells.


[back to table of contents](../../../README.md)