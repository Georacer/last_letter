## Using `last_letter` as the Physics Simulator for ArduPlane SITL

Follow the [ArduPlane SITL instructions](http://dev.ardupilot.com/wiki/setting-up-sitl-on-linux/) to download and compile the ArduPlane code.

When everything is done, enter the ArduPlane directory, usually using `cd ~/ardupilot/ArduPlane`
and execute the SITL, using the external simulator:
```
sim_vehicle.sh -e --console
```

This will run the ArduPlane code, MAVProxy and `last_letter` along with RViz all in one go. The RViz visualizer may take several seconds to start.
This procedure will automatically invoke the `last_letter` launch file and set the `ArduPlane` parameter to `true` so that the communication nodes between `last_letter` and `ArduPlane` are raised.
If `last_letter` communicates with ArduPlane correctly, the MAVProxy console should display 3D satellite fix and track 10 satellites.

You can issue commands or rc overrides using MAVProxy normally.

[back to table of contents](../../../README.md)