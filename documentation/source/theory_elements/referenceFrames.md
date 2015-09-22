##Frames of Reference and Conventions

####Frames of reference
In order to express the magnitude and orientation of the vectors of various quantities in aerospace systems, there is the need to define frames of reference in regard to which those quantites are expressed. These are 3-dimensional, orthogonal, right-handed axes systems with a fixed point of origin and a specific orientation.

There are four major frames:
1. The *Earth Centered Inetial (ECI)* frame. Its origin is in the center of the Earth, the x-axis is towards the meridian, and the z-axis towards the North Pole. As its name hints, this is usually considered an inertial frame, i.e. is considered stationary, disregarding the rotation of the Earth as insignificant. Usually, we use spherical coordinates to express quantities in this system which take the form of the well-known GPS cooridantes.

2. The *North-East-Down (NED)* frame. This is an (quasi-)inertial Cartesian frame, placed tangeantly in the surface of the Earth at an aribtrary point, usually the UAV launch point. Its x-axis extends Northwards, the y-axis Eastwards and the z-axis **downwards**, forming a right-handed orthonormal system. This results in the altitudes above the surface being negative.

3. The *Body Frame*. This frame is placed in the center of gravity of the UAV. Its x-axis extends forward along the longitudinal axis of the aircraft, the y-axis extends laterally to the right wing and the z-axis downwards. This is a rotating frame.

4. The *Wind Frame*. This frame is also placed in the center of gravity of the UAV. The x-axis faces towards the direction of the relative wind and results from the *Body Frame* by a rotation in the y-axis (by the angle of attack), followed by a rotation in the z-axis (by the angle of sidelsip).

Other frames are also used in `last_letter`, wich have auxilliary purpose:
5. The *Propeller Frames*. These are moving frames, placed on the propeller hubs. Their x-y plane coincide with the plane of rotation of the propeller. These planes are useful when a propeller motor is not aligned with the *Body Frame* and there is a need to transform the wind vector from the *Wind Frame* to the *Propeller Frame* and the propeller force and torque from the *Propeller Frame* to the *Body Frame*.

6. The *Airfoil Frames*. These are moving frames, aligned with the airfoils of the aircraft. When the whole aircraft is modeled as a single airfoil, these frames coincide with the *Body Frame*. However, when the aircraft is modeled as multiple airfoils (eg as a main wing and a stabilizer) or when there are rotating wings on the airframe (eg in the case of the V22-Osprey aircraft), the wind vector needs to be transformed from the *Wind Frame* to the *Airfoil Frame* and the airfoil force and torque from the *airfoil Frame* to the *Body Frame*.

####Positive control surface deflections
The following convention is used in `last_letter`, concerning the positive deflections of the control surfaces of fixed-wing aircraft. Each surface deflection is positive towards the direction that produces a positive moment in the corresponding axis. Thus:
 - Ailerons are deflected positively to the direction that produces a positive moment in the roll axis (roll right).
 - Elevator is deflected positively to the direction that produces a positive moment in the pitch axis (pitch up).
 - Rudder is deflected positively in the direction that produces a positive moment in the yaw axis (yaw right).

[back to table of contents](../../../README.md)