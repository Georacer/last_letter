##Ground Contact Point Parameters

This file contains information about the location of the contact points, their material and their spring-like behaviour.
The contact points are modeled as springs with damping to generate the normal force. Longitudinal and lateral friction coefficients are applied to that point to generate friction between the aircraft and the ground.
All of the followng parameters need to be prefixed with `airframe/`.

`groundReactionType`: This parameter selects the friction model.
- 0: No ground reactions. This selection disables the ground reactions and the aircraft falls throught the grounds.
- 1: This option selects the `PanosGroundReactions` class to model ground reactions. This model supports wheels and is more suitable for fixed-wing aircraft. However, static friction is inaccurate and the aircraft slides slightly when it rests unmoving on the ground. This model applies friction force as a function of the contact point velocity.
- 2: Point friction model. This model applies friction as a reaction to the rest of the forces that are exerted upon the aircraft, which has more accuracy in the case of static friction. Less slide is experienced than the `PanosGroundReactions` model. However, this rolling and steering wheels are not modelled in this class.

### No ground reactions
No further parameters need to be specified for this class

### Panos Ground reactions
The following parameters need to be specified when using this class:
`contactPtsNo`: The number of contact points you want to model on your aircraft.

`contactPoint#`: (Where # is the increasing, 1-indexed id of your contact point) This is a list of 6 elements, containing information about the contact point.
This information is:
1. `x coordinate`: The x coordinate of the location of the contact point in terms of the center of gravity, in the body frame.
2. `y coordinate`: The y coordinate of the location of the contact point in terms of the center of gravity, in the body frame.
3. `z coordinate`: The z coordinate of the location of the contact point in terms of the center of gravity, in the body frame.
4. `material type`: This parameter defines the material of the contact point. Available options are 0 for foam, 1 for metal, 2 for rubber wheel and 3 for composites.
5. `spring reaction constant`: The spring constant of the spring which will model the ground contact point.
6. `spring damping constant`: The damping constant of the spring which will model the ground contact point.
An example contact point specification is:
`contactPoint1: [0.162, -0.2324, 0.2214, 2.0, 500.0, 25.0]`

**Note:**Currently, the first 3 contact points are expected to be used for a tri-wheel configuration, with point no3 being the steerable nose gear. Landing gear is planned to be further parametrized.

`chanSteer`: The input channel which controls the steering wheel. Set to -1 to diable steering.

`steerAngle_max`: The maximum turning angle of the steering wheel.

`chanBrake`: The input channel which controls the brakes of the aircraft. Set to -1 for no braking action.

### Point friction
The following parameters need to be specified when using this class:
`contactPtsNo`: The number of contact points you want to model on your aircraft.

`contactPoint#`: (Where # is the increasing, 1-indexed id of your contact point) This is a list of 6 elements, containing information about the contact point.
This information is:
1. `x coordinate`: The x coordinate of the location of the contact point in terms of the center of gravity, in the body frame.
2. `y coordinate`: The y coordinate of the location of the contact point in terms of the center of gravity, in the body frame.
3. `z coordinate`: The z coordinate of the location of the contact point in terms of the center of gravity, in the body frame.
4. `material type`: This parameter defines the material of the contact point. Available options are 0 for foam, 1 for metal, 2 for rubber and 3 for composites.
5. `spring reaction constant`: The spring constant of the spring which will model the ground contact point.
6. `spring damping constant`: The damping constant of the spring which will model the ground contact point.
An example contact point specification is:
`contactPoint1: [0.162, -0.2324, 0.2214, 2.0, 500.0, 25.0]`

[back to table of contents](../../../README.md)