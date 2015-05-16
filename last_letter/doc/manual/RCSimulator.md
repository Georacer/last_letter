### RC Simulator

For this scenario, you will need to have a joystick (or similar device visible to the "joy" ROS package) plugged in to your machine. If this is the fist time you interface `last_letter` with an input device, you will need to configure it, according to [this guide](RCCal.md).

Running
```bash
roslaunch last_letter launcher.launch manualMode:=true
```
 will start the simulator running the `last_letter` standard aircraft, visible in the rviz visualizer, under direct, manual user control. It will also open the `rqt_dashboard` plugin for rqt_gui, which contains virtual flight instruments.

####Camera angle
Until now, there is no way to configure the camera position to stay fixed on the ground, like in most traditional RC Simulators. The available camera configurations are available in the `Views` panel of `rviz`, in the `Type` dropdown list.

[back to table of contents](../../../README.md)