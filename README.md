# last_letter : Unmanned Aerial Vehicle simulation and control packages

"last_letter" is a collection of [ROS](http://ros.org/) packages suitable for Unmanned Aerial Vehicle research, control algorithm development, non-linear physics simulation, Software-in-the-Loop (SIL) simulation and Hardware-in-the-Loop (HIL) simulation. It is developed by George Zogopoulos Papaliakos 
This document serves as a manual for this congregation of ROS packages.

## Table of Contents

- [Compatibility](#compatibility)
- [Installation](#installation)
- [Packages](#packages)
	- [last_letter](#last_letter)
	- [uav_utils](#uav_utils)
	- [mathutils](#mathutils)
	- [rqt_dashboard](#rqt_dashboard)
- [Important Messages](#important-messages)
- [How to Run](#how-to-run)
	- [Typical Scenarios](#typical-scenarios)
		 - [RC Simulator](#rc_simulator)
		- [A Simple Autopilot](#a-simple-autopilot)
- [Parameter Files](#parameter-files)
	- [Aircraft Parameters](#aircraft-parameters)
	- [Environment Parameters](#environment-parameters)
	- [Controller Parameters](#controller-parameters)
	- [Sensor Parameters](#sensor-parameters)
- [License](#license)
- [Contributors](#contributors)


## Compatibility

This collection of packages is being developed and used under Linux Ubuntu 12.04 using ROS Hydro.

## Installation

Just clone the root of this repo to your catkin workspace. Each of the package folders should be under your .../ROS/catkin_ws/src directory.

## Packages

### last_letter

This is the core package, containing the physics and kinematics simulator and controller nodes. 

## How to Run

The last_letter/launch file contains a few template .launch files which can be used to raise multiple nodes while reading from several parameter files of your choice. Consult them in order to build your own .launch files. The proposed structure is to split the simulation and visualization launch files, since these two functions are functionally independent anyway.
