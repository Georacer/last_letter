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
`ros2 launch last_letter default.launch.py uav_name:=<uav_name> world_file_path:=<world_path>`

## On Docker

### Building a Docker image

`cd` to the root directory and run `sudo docker build -t last_letter -f tools/Dockerfile --progress=tty .`.
It takes approximately 10 minutes.

### Run on Docker

`xhost +` (or `xhost +local:docker`?)
`sudo docker run -it --rm --volume=/tmp/.X11-unix/:/tmp/.X11-unix/ --device=/dev/dri:/dev/dri -e DISPLAY -p 14570:14570 -p 4560:4560 last_letter bash`

and then launch the launch file manually by
`. ros_ws/install/setup.bash`
`ros2 launch last_letter default.launch.py uav_name:=<uav_name> world_file_path:=<world_path>`
