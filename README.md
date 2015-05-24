# `last_letter` : Unmanned Aerial Vehicle simulation and control packages

`last_letter` is a collection of [ROS](http://ros.org/) packages suitable for Unmanned Aerial Vehicle research, control algorithm development, non-linear physics simulation, Software-in-the-Loop (SIL) simulation and Hardware-in-the-Loop (HIL) simulation. It is developed by George Zogopoulos-Papaliakos.
This document serves as a table of contents for the documentation of this congregation of ROS packages.

## Gitter chatroom
[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/Georacer/last_letter?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge) **(omnipresence not guaranteed)**

## Scientific background
Great effort has been exerted for the simulator to be as realistic as possible. To that goal, the project incorporates bibliography related to flight dynamics, physics modeling and sensor modeling. A congreggated document discussing the modeling of a fixed-wing UAV can be found in the [*Modeling a Fixed-Wing UAV*](https://github.com/Georacer/uav-modeling) Github project. It is a LaTeX project with a single .pdf output document.
Although `uav-modeling` is independent of `last_letter`, it will often be referred to in the documentation.

## Table of Contents

- [Compatibility](last_letter/doc/manual/compatibility.md)
- [Installation](last_letter/doc/manual/ll_installation.md)
- [Updating](last_letter/doc/manual/updating.md)

<!-- - [Included Packages](#packages)

- [Important Messages](#important-messages) -->

- [Configuring your input device](last_letter/doc/manual/RCCal.md)
- How to Run
    - [Launching and arguments](last_letter/doc/manual/launchingAndParams.md)
	- Typical Scenarios
		- [As simulator for the ArduPlane SITL](last_letter/doc/manual/ArduPlane_SITL.md)
        - [As an RC simulator](last_letter/doc/manual/RCSimulator.md)

<!-- [A Simple Autopilot](#a-simple-autopilot) -->

- [Frames of Reference and Conventions](last_letter/doc/manual/referenceFrames.md)
- [Parameter Files](last_letter/doc/manual/parameterFiles.md)
- [Logging](last_letter/doc/manual/logging.md)

<!-- Doxygen -->

<!-- Tutorials -->
<!-- - Creating your own aircraft -->

<!-- - [Controller Parameters](#controller-parameters) -->
<!-- - [Sensor Parameters](#sensor-parameters) -->

- [Contributors](last_letter/doc/manual/contributors.md)

<!-- ## Packages

### last_letter

This is the core package, containing the physics and kinematics simulator and controller nodes. -->

##License
The `last_letter` software project, which is a collection of ROS packages, uses the [GPLv3 license](http://choosealicense.com/licenses/gpl-3.0/).

##Screenshots
####Avionics panel and visualization of an aircraft pre-flight
![last letter initial screen](last_letter/doc/manual/figures/cover_init.png)

####An aircraft 3D model during take-off
![last letter initial screen](last_letter/doc/manual/figures/cover_takeoff.png)

####An indicative rqt_graph showing the node and topic layout of the simulator
![last letter initial screen](last_letter/doc/manual/figures/cover_rosgraph.png)