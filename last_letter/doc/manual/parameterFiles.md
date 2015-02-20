## ROS Parameter Files

Most of `last_letter`'s libraries and executables are initialized (or operate based) upon several parameters. These parameters are loaded onto the *parameter server* on simulation startup by the launch file.

**All numerical values of the parameters must be forced to a float format by a tralling `.0` if needed, unless otherwise stated**

The main categories of parameters are the following:
- **aircraft** - The corresponding parameter files can be found at the `last_letter/data/parameters/<aircraft_name>/` folder. The parameters are separated into the following subcategories:
    - [initialization](last_letter/doc/manual/aircraftInitParams.md)
    - [aerodynamics](last_letter/doc/manual/aircraftAeroParams.md)
    - [propulsion](last_letter/doc/manual/aircraftPropParams.md)
    - [ground reactions](last_letter/doc/manual/aircraftGroundParams.md)
    - [dashboard](last_letter/doc/manual/aircraftDashParams.md)
- [**environment**]
- [**world**]

[back to table of contents](../../Readme.md)