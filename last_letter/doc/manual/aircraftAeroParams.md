## Aircraft Aerodynamic Parameters

This file contains the parameters of the aerodynamics model.

The aerodynamics model assumes that all of the aircraft is fused into one lift, drag and moment model, ie separate wing surfaces are not modeled and calculated separately. This is not uncommon to the aircraft identification bibliography. Essentially, what this means is that you cannot provide NACA numbers for your main and tail wing and expect them to be incorporated. This is DATCOM turf!

There are 3 types of aerodynamics models available. These are selected with the variable

`aerodynamicsType`(**integer**)

- 0: Selects an empty aerodynamics model, which has no interactions with the surrounding air. Useful for modeling a rock falling in the void!
- 1: Standard Linear Aerodynamics - model with linear relation to its parameters. This is the most common and classic model found in bibliography, which models the reactions of the air using constant linear coefficients to the aircraft states.
- 2: Spline Aero - derivative class from 1, which models the lift and drag coefficients as cubic splines. This is useful if you have extracted the lift and drag curves of your aircraft through experiments or FDM simulation but can't quite capture it in a simple parametric model. Cubic splines offer you the possibility to import your data in full resolution.

### Standard Linear Aerodynamics Model
In general, none of the parameters corresponding to this aerodynamics model needs to be non-zero. By setting values to zero, the corresponding variable has no effect to the aircraft behaviour.

#### Coefficient of Lift in Relation to Angle of Attack Parameters
The coefficient of lift is modeled using primarily the lead of the book `Small Unmanned Aircraft: Theory and Practice, Randal W. Beard & Timothy W. McLain`. It is modeled as the weigthed sum of two parts, a linear function of angle-of-attack and the coefficient of lift of a flat plate. In the area withing the stall angle the first part dominates. Post-stall, the wing is modeled as a flat plate.
```C++
double sigmoid = ( 1+exp(-M*(alpha-alpha0))+exp(M*(alpha+alpha0)) ) / (1+exp(-M*(alpha-alpha0))) / (1+exp(M*(alpha+alpha0)));
double linear = (1.0-sigmoid) * (c_lift_0 + c_lift_a0*alpha); //Lift at small AoA
double flatPlate = sigmoid*(2*copysign(1,alpha)*pow(sin(alpha),2)*cos(alpha)); //Lift beyond stall

double result  = linear+flatPlate;
```

`alpha_stall` Stall angle-of-attack of the aircraft.

`c_lift_0` Zero angle-of-attack lift coefficient.

`c_lift_a` Angle-of-attack-related factor of the lift coefficient

`mcoeff` A factor related to how abrupt the swith-over between the linear and flat-plate model is.

#### Other Parameters Related to the Coefficient of Lift
`c_lift_q` Pitch-rate to lift coefficient

`c_lift_deltae` Elevator deflection to lift coefficient

#### Coefficient of Drag in Relation to Angle of Attack Parameters
The coefficient of drag is modeled as the sum of the parasitic and the induced drag:
```
double c_drag_a = c_drag_p + pow(c_lift_0+c_lift_a0*alpha,2)/(M_PI*oswald*AR);
```

`c_drag_p` The parasitic drag coefficient

#### Other Parameters Related to the Coefficient of Drag
`c_drag_q` Pitch-rate to drag coefficient

#### Other Parameters of the Standard Linear Model
`c_l_0` Bias rolling moment coefficient

`c_l_p` Rolling moment coefficient in relation to roll rate

`c_l_deltaa` Rolling moment coefficient in relation to aileron deflection (must be positive)

`c_l_deltar` Rolling moment coefficient in relation to rudder deflection

`c_l_r` Rolling moment coefficient in relation to yaw rate

`c_l_b` Rolling moment coefficient in relation to angle-of-sideslip

`c_m_0` Bias pitching moment coefficient

`c_m_a` Pitching moment coefficient in relation to angle-of-attack

`c_m_q` Pitching moment coefficient in relation to pitch rate

`c_m_deltae` Pitching moment coefficient in relation to elevator deflection (must be positive)

`c_n_0` Bias pitching moment coefficient

`c_n_b` Yawing moment coefficient in relation to angle-of-sideslip

`c_n_p` Yawing moment coefficient in relation to roll-rate

`c_n_r` Yawing moment coefficient in relation to yaw-rate

`c_n_deltaa` Yawing moment coefficient in relation to aileron deflection

`c_n_deltar` Yawing moment coefficient in relation to rudder deflection (must be positive)

`c_y_0` Bias side force coefficient

`c_y_b` Side force coefficient in relation to angle-of-attack

`c_y_p` Side force coefficient in relation to roll-rate

`c_y_r` Side force coefficient in relation to yaw-rate

`c_y_deltaa` Side force coefficient in relation to aileron deflection

`c_y_deltar` Side force coefficient in relation to rudder deflection

### Spline Aero Parameters
The following parameters are used to define polynomials or splines to describe the form of the coefficient of lift and drag as a function of angle-of-attack.

`cLiftPoly/polyType`(**integer**) The type of polynomial to be used for construcint coefficient of lift in relation to angle-of-attack. In this case valid options are 0 (1D polynomial) or 2 (cubic spline)

In the case where cubic spline is selected, the following parameters are required:
`cLiftPoly/breaksNo`(**integer**) The number of transition points of the coefficient of lift spline.

`cLiftPoly/breaks` The values of the transition points. The length of this list should be `breaksNo+1` with the first element acting as a lower bound for your expected range.

`cLiftPoly/coeffs` The array holding the cubic spline coefficients for each spline section. Each row has four elements, starting from the zero-order term up to the 3rd order term. This array has `4*breaksNo` elements.

`cDragPoly/polyType`, `cDragPoly/breaksNo`, `cDragPoly/breaks`, `cDragPoly/coeffs` Look above in the `cLiftPoly` parameters.

### Other Parameters
`s` Wing surface area

`b` Aircraft wingspan (from tip to tip)

`c` Mean chord length of the aircraft

`c_deltaa_max` Maximum aileron deflection (in radians)

`c_deltae_max` Maximum elevator deflection (in radians)

`c_deltar_max` Maximum rudder deflection (in radians)

[back to table of contents](../../../README.md)

