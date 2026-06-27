import os

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    world_name = LaunchConfiguration('world_name')
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='mtg_room.world',
        description='Name of the world file to load'
    )
    ld.add_action(declare_world_name)

    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = PathJoinSubstitution([
        FindPackageShare('sim_pkg'),
        'worlds',
        world_name
    ])

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('sim_pkg'),
                'models'))
    ld.add_action(set_env_vars_resources)

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    ld.add_action(gzserver_cmd)

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )
    ld.add_action(gzclient_cmd)

    return ld
