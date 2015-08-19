##Dashboard Configuration Parameters

Along with the aircraft model characteristics, you can specify the appearance of the `rqt` dashboard window, an avionics panel which appears each time you launch `last_letter`.
The panel is made up of multiple gauges (which are grouped in 3 columns) and each gauge has a similar description, which uses the following parameters:

`topic`: The ROS topic which this gauge should be bound to. This can be any topic under the current `roscore`. Optionally, this can be a topic under the `uav_name/dashboard` topic directory.
Available topics under this directory are:
`euler.x`: Roll angle (in degrees)
`euler.y`: Pitch angle (in degrees)
`euler.z`: Yaw angle (in degrees)
`airspeed`: Airspeed (in m/s)
`alpha`: Angle of attack (in degrees)
`beta`: Angle of sideslip (in degrees)
`climbRate`: Climb rate (in m/s)
`altitude`: GPS altitude (in m)
`rotorspeed`: Propeller_1 rotational speed (in RPM)

`length`: The length of the arc of the gauge (in degrees).

`end_angle`: The angle at which the gauge reaches its maximum value. 0 angle is at "3 o'clock" in a full circle and the positive direction is counter-clockwise.
Example: If you want the gauge to cap directly at 12 o'clock you should specify `end_angle: 90`.

`min`, `max`: The values of the gauge at its minimum and maximum point.

`main_points`: The number of tickmarks of the gauge.

`warning`: A list of pairs, which specify the interval at which the gauge flashes yellow, as a warning sign.
Example: In order to make the gauge flash yellow between at [1000, 2000] and [6000, 7000] specify `warning: [1000, 2000, 6000, 7000]`.
Leave empty (`[]`) for no warning interval.

`danger`: Similar to `warning`, but the gauge flashes red when the needle is in the specified interval.

`multiplier` (string): Text item, to depict the multiplier which should be used with the gauge reading.

`units` (string): Text item, which indicated the unit of the gauge quantity.

`description` (string): Text item, which carries a description of the quantity depicted in the gauge.

[back to table of contents](../../../README.md)