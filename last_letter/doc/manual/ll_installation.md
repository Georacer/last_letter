## Installation

Like any other ROS set of packages, `last_letter` and its companion library packages `uav_utils` and `math_utils` require the (Robotics Operating System)[http://www.ros.org/] to be present on your system.
If you have previous experience with ROS, installation is as simple as cloning the package files in your workspace.
If you are a ROS novice, the following instructions will try to guide you throught the process.

### ROS Version Restrictions
Currently, `last_letter` is developed under ROS Hydro version. It is not guaranteed that it runs smoothly or at all on any other ROS version.

### Operating System Restrictions
Since ROS Hyrdo is compatible only with Ubuntu 12.04, 12.10 and 13.04, you need an Ubuntu Linux machine running one of those Ubuntu versions. It is recommended that you use Ubuntu 12.04, since this is the platform where this package is developed right now.

### ROS Installation
Follow the (instructions on the ROS website)[http://wiki.ros.org/hydro/Installation/Ubuntu] to install ROS Hydro in your machine.

### Catkin Workspace Creation
Afterward, you need to create a ROS Workspace. This is where custom software packages (which are not part of the official ROS repository) are created, copied or edited and this is where the ROS compiler (called `catkin` after ROS version Groovy) looks for your code.
Follow (these instructions)[http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace] to create your workspace. They instruct you to create your workspace under your home folder, but it can also be created elsewhere. Just be sure you have full rights to this folder, so a subfolder in your home folder is a good choice.
Make sure `$ROS_PACKAGE_PATH` sees your `catkin_ws/src` folder.

### Cloning the last_letter Simulator files
Execute the following commands in your console:
```bash
roscd
cd ../src #Navigate in your ROS user source files directory
git clone https://github.com/Georacer/last_letter.git #Clone the simulator files
roscd
cd .. #Navigate in your ROS workspace
catkin_make #Compile the files
```
Compilation may take a while.

### Testing everything is installed properly
In a console, run:
```bash
roslaunch last_letter launcher.launch
```
This should start the simulator, and open a 3D simulation environment.
`rqt_gui` application will start as well, which is used to display the avionics instruments, but right now automatic configuration options do not work (WIP).
If everything went smoothly, you can close everything and proceed to the instruction manual to learn how to use the `last_letter` simulator.

[back to table of contents](../../../Readme.md)