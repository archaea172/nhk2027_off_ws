import os
import xacro

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    package_share = get_package_share_directory("sim_pkg")
    world = os.path.join(package_share, "worlds", "arm_plane.world")
    xacro_file = os.path.join(package_share, "urdf", "ide_arm.xacro")
    spawn_pose = (0.0, 0.0, 0.2, 0.0)

    doc = xacro.process_file(xacro_file, mappings={"use_sim": "true"})
    robot_desc = doc.toprettyxml(indent="  ")
    robot_description = {"robot_description": robot_desc}

    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(package_share)
    )
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
    # ld.add_action(gzclient_cmd)
    
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_1',
        arguments=[
            '/joint_sim1@std_msgs/msg/Float64]ignition.msgs.Double',
            '/joint_sim2@std_msgs/msg/Float64]ignition.msgs.Double',
            '/joint_sim3@std_msgs/msg/Float64]ignition.msgs.Double',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/world/nhk2026/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/world/nhk2026/model/robot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        remappings=[
            ('/world/nhk2026/model/robot/joint_state', '/joint_states'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    ld.add_action(bridge)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[],
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ],
    )
    ld.add_action(robot_state_publisher)
    create_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                '-name', 'robot',
                '-allow_renaming', 'false',
                '-x', str(spawn_pose[0]),
                '-y', str(spawn_pose[1]),
                '-z', str(spawn_pose[2]),
                '-Y', str(spawn_pose[3])
                ],
    )
    ld.add_action(create_node)

    foxglove_bridge_cmd = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml',
            )
        ),
        launch_arguments={
            'port': '8765',
            'use_sim_time': 'true',
        }.items(),
    )
    ld.add_action(foxglove_bridge_cmd)
    return ld
