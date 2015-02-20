## Aircraft Initialization Parameters

These are contained in the file `init.yaml` and describe basic initialization quantities of the aircraft.

`simRate`: The simulation frame-rate (in Hz)
`coordinates`: The initial coordinates of the aircraft in the WGS84 frame, in latitude, longitude and altitude (in degrees and meters respectively)
`position`: The initial position of the aircraft in the **NED** frame (in meters)
`orientation`: The initial orientation of the aircraft in quaternion form; angle is the last argument
`velLin` The initial linear velocity of the aircraft, in body axes (in meters/second)
`velAng` The initial angular velocity of the aircraft, in body axes (in radians/second)
`aileron` The initial aileron deflection, normalized in -1/1
`elevator` The initial elevator deflection, normalized in -1/1
`throttle` The initial throttle setting, normalized in 0/1
`rudder` The rudder aileron deflection, normalized in -1/1
