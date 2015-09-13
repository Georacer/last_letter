## Tutorial - Building your own airplane model

### Table of contents
- [Introduction](#introduction)
- [Preparing the files](#preparing-the-files)
- [Specifications](#specifications)
- [Rigid body parameters](#rigid-body-parameters)
- [Propulsion](#propulsion)
- [Aerodynamics](#aerodynamics)
- [Contact points](#contact-points)
- [Dashboard](#dashboard)
- [Initialization options](#initialization-options)
- [The URDF](#the-urdf)
- [Example values](#example-values)
- [Test flight and corrections](#test-flight-and-corrections)

### Introduction

In this tutorial you will learn how to build your own airplane model in `last_letter`, based on known measurements of specific characteristics. A flying-wing airplane will be modeled and simple models are used for aerodynamics and propulsion, but this is intentional, in order to provide an easy-to-follow guide.

The first part of this tutorial explains the aircraft model folder structure and contents.
Afterwards, the necessary input data in order to build the model is presented.
Finally, this data is used to calculate the airplane parameters, which `last_letter` reads from.

### Preparing the files

Each aircraft in `last_letter` has a dedicated, separate folder, found in the `last_letter/data/parameters/aircraft` directory. The name of the folder is the name of the aircraft, which for the purposes of this tutorial will be `tut_airplane` from now on.

Each time you create a new airplane, it is recommended that you copy the folder of this tutorial, rename the approprialte files and apply changes over them.

Inside the `tut_airplane` folder, 6 specific files and 1 specific folder must be present for the model to be valid, for use in `last_letter`:
1. The `rigidBody.yaml` file, which contains mass data of the airplane.
2. The `aerodynamics.yaml` file, which contains the aerodynamics parameters.
3. The `propulsion.yaml` file, which contains the propulsion plant information.
4. The `contactPts.yaml` file, which describes the aircraft points of contact with the ground.
5. The `dashboard.yaml` file, which specifies the appearance of the avionics panel.
6. The `init.yaml` file, which contains the initial values of the simulation.
You can find out more on each parameter in [this page](parameterFiles.md), but we'll cover the necessary information in this tutorial.

The `urdf` folder contains files relevant to the visual rendering of the aircaft. It must contain at least the `tut_airplane.urdf` file, which is named exactly like the airplane. [`.urdf` files](http://wiki.ros.org/urdf/Tutorials) are used by ROS to communicate the spacial configuration of the robot to the selected visualization software, which is usually RViz.
Naturally, a 3D model file of the aircraft is expected to be present, for the visualization software to display. `last_letter` uses the `.stl` file type and specifically reads from the `tut_airplane.stl` file, again with the same name as the airplane.

### Specifications

Before we come up with the aircraft parameters, we need to specify some numbers for the performance of the vehicle. In fact these specifications will seed the calculations for the aircraft parameters.

All units are expressed in the S.I. system.

Initally, we will make some assumptions for a few constants:
`g:` `9.81`
`rho:` `1.225`
`pi:` `3.14159`

The required specifications are:
1. `m`, mass of the aircraft (in kg)
2. `b`, wingspan (in m)
3. `c`, mean wing chord (in m)
4. `h`, aircraft mean height (in m)
5. `l`, aircraft length (in m)
6. `V_c`, cruise speed (in m/s)
7. `a_c`, cruise angle of attack (in rads)
8. `V_m`, max horizontal speed (in m/s)
9. `a_m`, angle of attack when max horizontal speed is attained (in rads)4
10. `F_b`, max propeller bench thrust (in Newtons)
11. `V_p`, propeller pitch speed (in m/s)
12. `t_r`, time to reach maximum roll rate (in s)
13. `p_m`, maximum roll rate (in rad/s)
14. `da_max`, maximum aileron deflection (in rads)
15. `t_p`, time to reach maximum pitch rate (in s)
16. `q_m`, maximum pitch rate (in rad/s)
17. `de_max`, maximum elevator deflection (in rads)
18. `t_y`, time to reach maximum yaw rate (in s)
19. `r_m`, maximum yaw rate (in rad/s)
20. `dr_max`, maximum rudder deflection (in rads)
21. `a_stall`, the stall angle of our airplane

After selecting those values according to the desired performance of our simulated airplane or the actual performance of a real-world airplane, we can proceed with the calculations.

First, we declare an auxiliary variable for the dynamic pressure.
`qbar:` `0.5*rho*V_c^2`

From now on, text in `verbatim` represents actual input-content that should be entered in the appropriate .yaml file for the parameters to take effect.

### Rigid body parameters

First off, we input the parameters related with the rigid body of the aircraft physics.

We define the mass of the aircraft
`airframe/m:` `m`

and then the elements of the moment of inertia for each primary axis. The formula for the rectangular parallepiped is used (source: http://scienceworld.wolfram.com/physics/MomentofInertiaRectangularParallelepiped.html), as this shape resembles our flying wing to an acceptable degree.
**Don't input the forumla into the .yaml file. Instead write its result directly.**
`airframe/j_x:` `1/12*(b^2+h^2)*m`
`airframe/j_y:` `1/12*(c^2+h^2)*m`
`airframe/j_z:` `1/12*(c^2+b^2)*m`

Often times, the x-z off-diagonal element of the matrix of inertia is non-zero in airplanes, resulting to roll-yaw coupling, but for this simple example we set it to zero.
`airframe/j_xz:` `0.0`

### Propulsion

Next, we define the propulsion characteristics.

Our aircraft has only one thruster, which is a general term for any configuration which produces thrust. In our case it is a motor-propeller combinations.
`motor/nMotors:` `1`

The input channel which will control the motor should be the 2nd one. We will use the "AETR" convention.
`motor1/chanMotor:` `2`

The thruster end-effector (in our case the propeller) should be a little to the back of the center of gravity (CoG) of the airplane, but on the longitudinal axis. This parameter affects the visual location of the propeller in the 3D model.
`motor1/CGOffset:` `[-0.25, 0.0, 0.0]`

The propeller is not tilted in respect to the Body Frame at all. We set the corresponding roll-pitch-yaw angles to zero.
`motor1/mountOrientation:` `[0.0, 0.0, 0.0]`

The thruster is also not gimballed. On the contrary it is always poiting to a fixed direction. Thus, we disable the gimbal channel by setting it to -1 and leave the maximum gimbal angle specification to zero.
`motor1/chanGimbal:` `-1`
`motor1/gimbalAngle_max:` `0.0`

We can define the propeller rotation direction by choosing 1 or -1 in the next parameter. This affects the visuals on our plane, but also the direction of the propeller counter-torque, if the model actually produces such torque.
`motor1/rotationDir:` `1`

Now, we want to choose the type of the physics model that will implement the thruster itself. We will go with the "Beard Engine" (`BeardEng` in the source code), since it is a simple model to describe but also provides some realism. The corersponding model ID is 1. From now on, all the thruster parameters are specific to the Beard Engine model.
`motor1/motorType:` `1`

We fill in the `k_motor` paremeter with the propeller pitch speed.
`motor1/k_motor:` `V_p`

And based on the bench thrust of the propeller (which on the condition that the propeller doesn't stall is the maximum thrust) we calculate the `s_prop` parameter.
`motor1/s_prop:` `2*F_b/(rho*k_motor^2)`

The next parameter specifies the amount of torque the propeller produces proportionally to the square of the propeller rotational velocity. We don't want the thruster to produce any counter torque, so we set the next parameter to zero.
`motor1/k_t_p:` `0.0`

We specify the relation between the throttle input and the motor RPM. We choose that on full throtle, the motor will run at 254 rads per second.
`motor1/k_omega:` `254`

The last parameter is a dummy one and can always be set to 1.
`motor1/c_prop:` `1.0`

### Aerodynamics

Along with the thruster model, the aerodynamics model is the part which encapsulates most of the performance and "character" of the airplane. Getting this part right is crucial, but this doesn't need to happen in the first iteration. It is very natural to set some initial values, test fly the model and then come back and change the numbers so that the airplane "feels right".

First of all, we will model our airplane as a single wing, which represents the composed, overall aerodynamic behaviour. Our airplane **is** one wing after all.
`airfoil/nWings:` `1`

We place the center of lift (CoL) of the wing a bit behind the CoG. This is a known practice in aeromodeling to ensure a stable flying platform and is depicted in `last_letter` as well.
`airfoil1/CGOffset:` `[-0.02, 0.00, 0.0]`

We won't rotate the wing away from the Body Frame. If we wanted to add some angle of incidence, this is where we would do it.
`airfoil1/mountOrientation:` `[0.0, 0.0, 0.0]`

`last_letter` supports rotating wings in one axis, but our basic model won't incorporate any such action. We disable the wing gimbal control channel.
`airfoil1/chanGimbal:` `-1`

Since we set the `chanGimbal` parameter to -1, the other two options have no effect.
`airfoil1/gimbalAxis:` `0`
`airfoil1/gimbalAngle_max:` `0.0`

Now, we want to choose the common aerodynamics model, where the response of the aircraft on various quantities is modeled as a linear function with given coefficients, pre-multiplied with the dynamic pressure. This approach is extensively used in airplane stability and control litterature.
`airfoil1/aerodynamicsType:` `1`

Input the geometric characteristics of the wing.
The wing surface.
`airfoil1/s:` `b*c`
The wingspan.
`airfoil1/b:` `b`
And the mean chord.
`airfoil1/c:` `c`

Next, we go on to the meat of the aerodynamics model. We start with the lift parameters and we calculate the static lift of the wing.
`airfoil1/c_lift_0:` `2*m*g / ( (1+10*a_c)*rho*V_c^2*s)`

We don't want the elevator to produce any lift.
`airfoil1/c_lift_deltae:` `0`

We will set the coefficient of lift in respect to the angle of attack (AoA) quasi-arbitrarily, as 10 times the static lift. You can change that value if you are better informed, but keep in mind that the rest of the calculations are based on this assumption.
`airfoil1/c_lift_a:` `10*c_lift_0`

We don't want pitch rate to affect lift.
`airfoil1/c_lift_q:` `0`

This coefficient is internal to this aerodynamics model and affects the transitory behaviour of the wing, from lift to stall. In general, you don't need to change this number.
`airfoil1/mcoeff:` `50`

The oswald number is related to the air properties and affects the induced drag. You don't need to change this parameter.
`airfoil1/oswald:` `0.9`

Set the stall angle of your airplane, in radians.
`airfoil1/alpha_stall:` `a_stall`


`airfoil1/c_drag_q:` `0`
`airfoil1/c_drag_deltae:` `0`
`airfoil1/c_drag_p:` `s_prop*(k_motor^2 - V_m^2)/(V_m^2*s) - (c_lift_0*(1+10*a_m))^2/(pi*oswald*b^2/s)`
`airfoil1/c_y_0:` `0`
`airfoil1/c_y_b:` `-l*h/s * 0.8/(pi/2)`
`airfoil1/c_y_p:` `0`
`airfoil1/c_y_r:` `0`
`airfoil1/c_y_deltaa:` `0`
`airfoil1/c_y_deltar:` `c_y_b/5`

`airfoil1/c_l_0:` `0`
`airfoil1/c_l_p:` `-j_x/(qbar*s*b*t_r/3)`
`airfoil1/c_l_b:` `C_l_p/10`
`airfoil1/c_l_r:` `0`
`airfoil1/c_l_deltaa:` `j_x*p_max/(qbar*s*b*t_r/3*da_max)`
`airfoil1/c_l_deltar:` `0`
`airfoil1/c_m_a:` `-0.5`
`airfoil1/c_m_0:` `-c_m_a*a_c`
`airfoil1/c_m_q:` `-j_y/(qbar*s*c*t_p/3)`
`airfoil1/c_m_deltae:` `j_y*q_max/(qbar*S*c*t_p/3*de_max)`
`airfoil1/c_n_0:` `0`
`airfoil1/c_n_b:` `0.25`
`airfoil1/c_n_p:` `0`
`airfoil1/c_n_r:` `-j_z/(qbar*S*b*t_y/3)`
`airfoil1/c_n_deltaa:` `0`
`airfoil1/c_n_deltar:` `I_z*r_max/(qbar*S*b*t_y/3*dr_max)`

`airfoil1/deltaa_max:` `da_max`
`airfoil1/deltae_max:` `de_max`
`airfoil1/deltar_max:` `dr_max`

`airfoil1/chanAileron:` `0`
`airfoil1/chanElevator:` `1`
`airfoil1/chanRudder:` `3`

### Contact points

`dx:` `0.05`
`k:` `m*g/(2*dx)`
`d:` `2*0.6*sqrt(k*m)`

`airframe/groundReactionType:` `1`
`airframe/contactPtsNo:` `7`
`airframe/contactPoint1:` `[0.162, -0.2324, 0.2214, 2.0, k, 3.5]`
`airframe/contactPoint2:` `[0.162, 0.2324, 0.2214, 2.0, k, 3.5]`
`airframe/contactPoint3:` `[-0.8639, 0.0, 0.0522, 2.0, k, 3.5]`
`airframe/contactPoint4:` `[-0.0832, 0.9671, -0.1283, 0.0, k, 3.5]`
`airframe/contactPoint5:` `[-0.0832, -0.9671, -0.1683, 0.0, k, 3.5]`
`airframe/contactPoint6:` `[0.3785, 0.0, 0.017, 0.0, k, 3.5]`
`airframe/contactPoint7:` `[-0.9328, 0.0, -0.2196, 0.0, k, 3.5]`

`airframe/chanSteer:` `4`
`airframe/steerAngle_max:` `0.79`
`airframe/chanBrake:` `5`

### Dashboard

Yaw gauge
`Gauge1:`
`    topic: dashboard/euler/z`
`    length: 360.0`
`    end_angle: -90.0`
`    min: -180.0`
`    max: 180.0`
`    main_points : 8`
`    warning: []`
`    danger: []`
`    multiplier: ''`
`    units: degrees`
`    description: Yaw`

Roll gauge

Pitch gauge

Airspeed gauge

Climb rate gauge

Altitude gauge

Alpha gauge

Beta gauge

Rotorspeed gauge

### Initialization options

`init/coordinates:` `[lat, lon, g_alt]`
`init/position:` `[0.0, 0.0, -0.3]`
`init/orientation:` `[-0.0000, 0.0039, 0.0000, 1.0000]`
`init/velLin:` `[0.0, 0.0, 0.0]`
`init/velAng:` `[0.0, 0.0, 0.0]`
`init/aileron:` `0`
`init/elevator:` `0`
`init/throttle:` `0`
`init/rudder:` `0`
`init/chanReset:` `9`

### The URDF



### Example values

`m:` `1.5`
`b:` `1.5`
`c:` `0.2`
`h:` `0.1`
`l:` `0.40`
`a_c:` `deg2rad(2)`
`V_c:` `55/3.6`
`a_m:` `deg2rad(0)`
`V_m:` `80/3.6`
`F_b:` `13`
`V_p:` `100/3.6`
`t_r:` `2`
`p_max:` `pi`
`da_max:` `deg2rad(20)`
`t_p:` `4`
`q_max:` `deg2rad(90)`
`de_max:` `deg2rad(20)`
`t_y:` `4`
`r_max:` `deg2rad(20)`
`dr_max:` `deg2rad(20)`
`a_stall:` `0.3491`

### Test flight and corrections