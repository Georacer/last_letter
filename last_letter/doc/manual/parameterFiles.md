## ROS Parameter Files

Most of `last_letter`'s libraries and executables are initialized (or operate based) upon several parameters. These parameters are loaded onto the *parameter server* on simulation startup by the launch file. The order in which the parameters are declared in the corresponding `*.yaml` file is not important.

**All numerical values of the parameters must be forced to a float format by a trailling `.0` if needed, unless otherwise stated**

The main categories of parameters are the following:
- **aircraft** - The corresponding parameter files can be found at the `last_letter/data/parameters/<aircraft_name>/` folder. The parameters are separated into the following subcategories:
    - [initialization](aircraftInitParams.md)
    - [rigid body](rigidBodyParams.md)
    - [aerodynamics](aircraftAeroParams.md)
    - [propulsion](aircraftPropParams.md)
    - [ground reactions](aircraftGroundParams.md)
    - [dashboard](aircraftDashParams.md)
- [**environment**]
- [**world**]

[back to table of contents](../../../README.md)