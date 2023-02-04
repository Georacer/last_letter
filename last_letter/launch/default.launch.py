import os

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

MODELS_FOLDER = os.path.expanduser("~/last_letter_models/")
os.environ["GAZEBO_PLUGIN_PATH"] += os.pathsep + os.path.expanduser("~/.gazebo/plugins")
os.environ["GAZEBO_MODEL_PATH"] += os.pathsep + os.path.join(MODELS_FOLDER, 'models')
os.environ["GAZEBO_RESOURCE_PATH"] += os.pathsep + os.path.join(MODELS_FOLDER, 'worlds')

def generate_launch_description():

    uav_name_arg = DeclareLaunchArgument(name='uav_name', default_value='skywalker_2013', description='The UAV model name.')
    uav_name_value = LaunchConfiguration('uav_name')
    world_file_path_arg = DeclareLaunchArgument(name='world_file_path',
                                                default_value=[uav_name_value, '.world'],
                                                description='The world file to load.')
    world_file_path_value = LaunchConfiguration('world_file_path')

    # Create a log_level argument for the launcher, trickling down to all nodes
    logger = launch.substitutions.LaunchConfiguration("log_level")
    logging_config = launch.actions.DeclareLaunchArgument(
        "log_level", default_value=["info"], description="Logging level"
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzserver.launch.py",
            )
        ),
        launch_arguments={
            "world": world_file_path_value,
            "verbose": "true",
            "lockstep": "true",
            "server_required": "true",
            "init": "true",
            "factory": "false",
            "force_system": "false",
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzclient.launch.py",
            )
        ),
        launch_arguments={
            "world": world_file_path_value,
            "verbose": "true",
            "lockstep": "true",
            # "server_required": "true",
            "init": "true",
            "factory": "false",
            "force_system": "false",
        }.items(),
    )

    last_letter_node = Node(
        package="last_letter",
        namespace="",
        executable="uav_model_ros",
        name="uav_model_ros",
        parameters=[{"use_sim_time": True, "uav_name": uav_name_value}],
        arguments=["--ros-args", "--log-level", logger],
        output={"stdout": "screen"},
        # parameters=[{"uav_name": "skywalker"}],
        # prefix=["xterm -e gdb -ex run --args"],
    )

    px4_interface_node = Node(
        package="last_letter",
        namespace="",
        executable="px4_sitl_interface",
        name="px4_sitl_interface",
        parameters=[{"use_sim_time": True}],
        # prefix=["xterm -e gdb -ex run --args"],
        arguments=["--ros-args", "--log-level", logger],
        output={"stdout": "screen"},
    )

    node_parameters = os.path.expanduser("~/last_letter_models/HID.yaml")
    joy2chan_node = Node(
        package="last_letter",
        namespace="",
        executable="joy2chan",
        name="joy2chan",
        parameters=[{"use_sim_time": True}, node_parameters],
        # remappings=[("rawPWM", "ctrlPWM")],
    )

    joystick_driver = Node(
        package="joy",
        namespace="",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {
                "use_sim_time": True,
                "dev": "/dev/input/js0",
                "deadzone": 0.05,
            }
        ],
    )

    rosbag = launch.actions.ExecuteProcess(
        cmd = ['ros2', 'bag', 'record', '-a']
    )

    return LaunchDescription(
        [
            uav_name_arg,
            world_file_path_arg,
            logging_config,
            last_letter_node,
            px4_interface_node,
            gazebo_server,
            gazebo_client,
            joy2chan_node,
            joystick_driver,
            # rosbag,
        ]
    )
