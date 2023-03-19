# Instructions

## On bare system

### Building with ROS2

Have ROS2 installed in the system.
Create a `ros_ws/src` folder.
`cd` to `ros_ws`.
`colcon build --symlink-install`

### Run on bare system

Launch the launch file manually by
`. ros_ws/install/setup.bash`
`cd src/last_letter`
`make run USE_DOCKER=false`

## On Docker

### Building a Docker image

`cd` to the root directory and run `sudo docker build -t last_letter -f tools/Dockerfile --progress=tty .`.
It takes approximately 10 minutes.

### Run on Docker

`cd src/last_letter`
`make run`

## Available commands

`make help`
