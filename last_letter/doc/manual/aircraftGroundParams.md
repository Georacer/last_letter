##Ground Contact Point Parameters

This file contains information about the location of the contact points, their material and their spring-like behaviour.
The contact points are modeled as springs with damping to generate the normal force. Longitudinal and lateral friction coefficients are applied to that point to generate friction between the aircraft and the ground.

`groundReactionType`: Set to 0 if you want to disable the ground reactions. The aircraft will fall through the ground. Set to 1 to enable ground reactions and use the `PanosGroundReactions` class to model them.

`contactPtsNo`: The number of contact points you want to model on your aircraft.

`contactPoint#`: (Where # is the increasing, 1-indexed id of your contact point) This is a list of 6 elements, containing information about the contact point.
This information is:

1. `x coordinate`: The x coordinate of the location of the contact point in terms of the center of gravity, in the body frame.
2. `y coordinate`: The y coordinate of the location of the contact point in terms of the center of gravity, in the body frame.
3. `z coordinate`: The z coordinate of the location of the contact point in terms of the center of gravity, in the body frame.
4. `material type`: This parameter defines the material of the contact point. Available options are 0 for foam, 1 for metal, 2 for rubber wheel and 3 for composites.
5. `spring reaction constant`: The spring constant of the spring which will model the ground contact point.
6. `spring damping constant`: The damping constant of the spring which will model the ground contact point.

**Note:**Currently, the first 3 contact points are expected to be used for a tri-wheel configuration, with point no3 being the steerable nose gear. Landing gear is planned to be further parametrized.

[back to table of contents](../../../README.md)