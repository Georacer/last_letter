# `last_letter` : Unmanned Aerial Vehicle simulation and control packages

`last_letter` is a collection of [ROS](http://ros.org/) packages suitable for Unmanned Aerial Vehicle research, control algorithm development, non-linear physics simulation, Software-in-the-Loop (SIL) simulation and Hardware-in-the-Loop (HIL) simulation. It is developed by George Zogopoulos-Papaliakos.
This document serves as a table of contents for the documentation of this congregation of ROS packages.

## Gitter chatroom
[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/Georacer/last_letter?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge) **(omnipresence not guaranteed)**

## Scientific background
Great effort has been exerted for the simulator to be as realistic as possible. To that goal, the project incorporates bibliography related to flight dynamics, physics modeling and sensor modeling. A congreggated document discussing the modeling of a fixed-wing UAV can be found in the [*Modeling a Fixed-Wing UAV*](https://github.com/Georacer/uav-modeling) Github project. It is a LaTeX project with a single .pdf output document.
Although `uav-modeling` is independent of `last_letter`, it will often be referred to in the documentation.

## Documentation

You can find full documentation for `last_letter` in the form of a project Github page here: http://georacer.github.io/last_letter/index.html

##License
The `last_letter` software project, which is a collection of ROS packages, uses the [GPLv3 license](http://choosealicense.com/licenses/gpl-3.0/).

##Screenshots
####Avionics panel and visualization of an aircraft pre-flight
![last letter initial screen](last_letter/doc/manual/figures/cover_init.png)

####An aircraft 3D model during take-off
![last letter initial screen](last_letter/doc/manual/figures/cover_takeoff.png)

####An indicative rqt_graph showing the node and topic layout of the simulator
![last letter initial screen](last_letter/doc/manual/figures/cover_rosgraph.png)