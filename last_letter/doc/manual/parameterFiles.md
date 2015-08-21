## ROS Parameter Files

Most of `last_letter`'s libraries and executables are initialized (or operate based) upon several parameters. These parameters are loaded onto the *parameter server* on simulation startup by the launch file. The order in which the parameters are declared in the corresponding `*.yaml` file is not important, but declaring the same parameter twice in separate locations/files will overwrite the previous value.

**All numerical values of the parameters within lists (in [] brackets) must be forced to a float format by a trailling `.0` if needed, unless otherwise stated**

The main categories of parameters are the following:
- **aircraft** - The corresponding parameter files can be found at the `last_letter/data/parameters/<aircraft_name>/` folder. The parameters are separated into the following subcategories:
    - [initialization](aircraftInitParams.md)
    - [rigid body](aircraftBodyParams.md)
    - [aerodynamics](aircraftAeroParams.md)
    - [propulsion](aircraftPropParams.md)
    - [ground reactions](aircraftGroundParams.md)
    - [dashboard](aircraftDashParams.md)
    - There is also a *urdf* folder in each aircraft folder, which contains the visual 3D model file and the *.urdf* file which handles it. More information on this folder will be given in another tutorial.
- [**HID**](HIDParams.md)
- [**environment**](environmentParams.md)
- [**world**](worldParams.md)

[back to table of contents](../../../README.md)