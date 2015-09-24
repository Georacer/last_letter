Aircraft Aerodynamic Parameters
===============================

This file contains the parameters of the aerodynamics model.

The aerodynamics model assumes that the aircraft aerodynamic response can be modeled as the some of a number of airfoils. This method allows for various common modelling approaches such as:

* Fusing the aircraft aerodynamics into one lift, drag and moment airfoil model, ie separate wing surfaces are not modeled and calculated separately. This is not uncommon to the aircraft identification bibliography
* Specifying different airfoils for the main wing and the stabilizer, each with its own aerodynamics model and location on the airframe relative to the Center of Gravity. Their contributions are summed up to provide the overall response of the aircraft.

``nWings``: The total number of airfoils that the aircraft contains.

.. note::

	The following parameters are referring to a single airfoil. They are groupped under the prefix ``airfoilX/``, where ``X`` is the number-id of the airfoil, starting from 1 for a single-airfoil aircraft (eg ``airfoil1/CGOffset``).

``CGOffset``: The location of the center of lift in terms of the center of gravity, in X/Y/Z body frame coordinates. Negative X means that the CoL is behind the CoG. Positive Y means that the CoL is to the right of the CoG. Negative Z means that the CoL is above the CoG.

``mountOrientation``: The rotation Roll-Pitch-Yaw (in this sequence) rotation angles which rotate the airfoil from the Body Frame to its actual Airfoil Frame fixture. If the whole aircraft is modelled as a single airfoil, this should be [0.0,0.0,0.0]

``chanGimbal``: The input channel which controls the airfoil gimbal. Set to -1 for no gimballing action.

``gimbalAxis``: Selects the axis of the `Airfoil Frame <../theory_elements/rerenceFrames.html>`_ around which the gimbal rotation is applied. Available options are:
0 - x-axis, the axis collinear with the wing chord and towards the front
1 - y-axis, the axis collinear with the wing span and towards the right
2 - z-axis, the remaining, orthogonal axis for a right-handed system, perpendicular to the x-y plane and downwards.

``gimbalAngle_max``: The maximum throw angle (in radians) that the gimbal is providing when given full input.

There are 3 types of airfoil aerodynamic models available. These are selected with the variable
``aerodynamicsType``:(**integer**)
- 0: Selects an empty aerodynamics model, which has no interactions with the surrounding air. Useful for adding a dummy wing on your aircraft, or modeling a rock falling in the void!
- 1: Standard Linear Aerodynamics - model with linear relation to its parameters. This is the most common and classic model found in bibliography, which models the reactions of the air using constant linear coefficients to the aircraft states.
- 2: Spline Aero - derivative class from 1, which models the lift and drag coefficients as cubic splines. This is useful if you have extracted the lift and drag curves of your aircraft through experiments or FDM simulation but can't quite capture it in a simple parametric model. Cubic splines offer you the possibility to import your data in full resolution.

No aerodynamics model
---------------------

No parameters need to be defined regarding this model.

Standard linear aerodynamics model
----------------------------------

In general, none of the parameters corresponding to this aerodynamics model needs to be non-zero. By setting values to zero, the corresponding variable has no effect to the aircraft behaviour.

Coefficient of lift in relation to angle of attack parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

The coefficient of lift is modeled using primarily the lead of the book *Small Unmanned Aircraft: Theory and Practice, Randal W. Beard & Timothy W. McLain*. It is modeled as the weigthed sum of two parts, a linear function of angle-of-attack and the coefficient of lift of a flat plate. In the area withing the stall angle the first part dominates. Post-stall, the wing is modeled as a flat plate.

.. code-block:: c++

	double sigmoid = ( 1+exp(-M*(alpha-alpha0))+exp(M*(alpha+alpha0)) ) / (1+exp(-M*(alpha-alpha0))) / (1+exp(M*(alpha+alpha0)));
	double linear = (1.0-sigmoid) * (c_lift_0 + c_lift_a0*alpha); //Lift at small AoA
	double flatPlate = sigmoid*(2*copysign(1,alpha)*pow(sin(alpha),2)*cos(alpha)); //Lift beyond stall

	double result  = linear+flatPlate;

``alpha_stall``: Stall angle-of-attack of the aircraft.

``c_lift_0``: Zero angle-of-attack lift coefficient.

``c_lift_a``: Angle-of-attack-related factor of the lift coefficient.

``oswald``: `Oswald efficiency factor <http://en.wikipedia.org/wiki/Oswald_efficiency_number>`_, related to the efficiency of the wing geometry. Used in the `Standard linear aerodynamics model`.

``mcoeff``: A factor related to how abrupt the swith-over between the linear and flat-plate model is.

Other parameters related to the coefficient of lift
+++++++++++++++++++++++++++++++++++++++++++++++++++

``c_lift_q``: Pitch-rate to lift coefficient

``c_lift_deltae``: Elevator deflection to lift coefficient

Coefficient of drag in relation to angle of attack parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

The coefficient of drag is modeled as the sum of the parasitic and the induced drag:

.. code-block:: c++

	double c_drag_a = c_drag_p + pow(c_lift_0+c_lift_a0*alpha,2)/(M_PI*oswald*AR);

``c_drag_p``: The parasitic drag coefficient

Other parameters related to the coefficient of drag
+++++++++++++++++++++++++++++++++++++++++++++++++++

``c_drag_q``: Pitch-rate to drag coefficient

Other parameters of the standard linear model
+++++++++++++++++++++++++++++++++++++++++++++

``c_l_0``: Bias rolling moment coefficient

``c_l_p``: Rolling moment coefficient in relation to roll rate

``c_l_deltaa``: Rolling moment coefficient in relation to aileron deflection (must be positive)

``c_l_deltar``: Rolling moment coefficient in relation to rudder deflection

``c_l_r``: Rolling moment coefficient in relation to yaw rate

``c_l_b``: Rolling moment coefficient in relation to angle-of-sideslip

``c_m_0``: Bias pitching moment coefficient

``c_m_a``: Pitching moment coefficient in relation to angle-of-attack

``c_m_q``: Pitching moment coefficient in relation to pitch rate

``c_m_deltae``: Pitching moment coefficient in relation to elevator deflection (must be positive)

``c_n_0``: Bias pitching moment coefficient

``c_n_b``: Yawing moment coefficient in relation to angle-of-sideslip

``c_n_p``: Yawing moment coefficient in relation to roll-rate

``c_n_r``: Yawing moment coefficient in relation to yaw-rate

``c_n_deltaa``: Yawing moment coefficient in relation to aileron deflection

``c_n_deltar``: Yawing moment coefficient in relation to rudder deflection (must be positive)

``c_y_0``: Bias side force coefficient

``c_y_b``: Side force coefficient in relation to angle-of-attack

``c_y_p``: Side force coefficient in relation to roll-rate

``c_y_r``: Side force coefficient in relation to yaw-rate

``c_y_deltaa``: Side force coefficient in relation to aileron deflection

``c_y_deltar``: Side force coefficient in relation to rudder deflection

Spline aero parameters
----------------------

The following parameters are used to define polynomials or splines which describe the coefficient of lift and drag as a 1-D function of the angle-of-attack. Two parameter groups are required, ``cLiftPoly`` and ``cDragpoly``.

``cLiftPoly``: A 1-variable (1D) polynomial describing the coefficient of lift (non-dimensional), as a function of angle of attack (in radians). This is a parameter group and all the related polynomial parameters must be set as a parameter under this group. A spline polynomial spanning the whole [-pi, pi] interval is suggested.
More information on the related parameters can be found in the [polynomial parameters page](polynomialParams.md).

``cDragpoly``: A 1-variable (1D) polynomial describing the coefficient of drag (non-dimensional), as a function of angle of attach (in radians). This is a parameter group and all the related polynomial parameters must be set as a parameter under this group. A spline polynomial spanning the whole [-pi, pi] interval is suggested.
More information on the related parameters can be found in the [polynomial parameters page](polynomialParams.md).

Other parameters
----------------

``s``: Wing surface area

``b``: Aircraft wingspan (from tip to tip)

``c``: Mean chord length of the aircraft

``c_deltaa_max``: Maximum aileron deflection (in radians)

``c_deltae_max``: Maximum elevator deflection (in radians)

``c_deltar_max``: Maximum rudder deflection (in radians)

``chanAileron``: The input channel which controls the ailerons. Set to -1 for no aileron effect.

``chanElevator``: The input channel which controls the elevator. Set to -1 for no elevator effect.

``chanRudder``: The input channel which controls the rudder. Set to -1 for no rudder effect.
