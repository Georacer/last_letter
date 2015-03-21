##Launching and Arguments

Prior to launching `last_letter` you may want to edit the parameter files for the aircraft you are about to use, as well as the `environment.yaml` file. These control the initial conditions of the simulation and are explained in the [parameters section](parameterFiles.md).
If you don't know how to handle the parameter files, it is perfecty ok to leave them as they are for your first flights, to get a feel of the program.

In order to launch the `last_letter` simulator a ROS *launch file* is used, which invokes the rest of the executables and organizes the simulation.

To launch the simulation, from any directory of your system enter
```bash
roslaunch last_letter launch.launcher <args>
```
This will raise the simulation executable, the `rqt` avionics panel and the `rviz` visualizer.

Arguments are passed to control the simulation behaviour and operational mode, with the syntax `<argname>:=<argvalue>`.

###Available arguments are:

**Operation mode arguments (mutually exclusive!)**

`manualMode`: **(default false)** Control the vehicle via full manual control, using a joystick or gamepad connected to your machine, which is read by the ROS core.

`autoMode`: **(default false)** Insert a controller in the loop. Joystick commands are forwarded to the controller, while the controller output feeds the aircraft model inputs. This mode is experimental and not recommended.

`ArduPlane`: **(default false)** Raise nodes which establish UDP connection with the ardupilot SITL and configure the aircraft inputs to be controlled by ArduPlane. It is recommended to not set this argument manually, but rather launch the ArduPlane SITL all in one go, by following [these instructions](ArduPlane_SITL.md).

**Logging**

`log`: **(deault false)** Setting this to `true` will request ROS to log all of your messages in a [bag file](http://wiki.ros.org/Bags) and save them into the file `UAV_recording.bag` found in `~/.ros`
See also the [logging](logging.md) section.

**Aircraft-related arguments**

`uav_name`: **(default HCUAV)** The name of the UAV you are about to fly. This corresponds and must be identical to a folder in `last_letter/data/parameters/aircraft`. The configuration and specification files of the aircraft are found in this folder.

[back to table of contents](../../../README.md)