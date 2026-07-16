import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gz_args = LaunchConfiguration('gz_args')
    start_foxglove = LaunchConfiguration('start_foxglove')
    pkg_share = FindPackageShare('diffbot_description')

    controllers_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'diffbot_ros2_control.yaml',
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([pkg_share, 'urdf', 'diffbot.urdf.xacro']),
            ' use_ros2_control:=true',
            ' controllers_file:=', controllers_file,
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'diffbot',
            '-topic', 'robot_description',
        ],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diffbot_base_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen'
    )

    twist_relay = Node(
        package='diffbot_description',
        executable='twist_to_twist_stamped.py',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/cmd_vel',
            'output_topic': '/diffbot_base_controller/cmd_vel',
            'frame_id': 'base_link',
        }]
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        output='screen',
        condition=IfCondition(start_foxglove)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'gz_args',
            default_value='-s -r empty.sdf',
            description='Gazebo arguments. Use "-r empty.sdf" to open the GUI.'
        ),
        DeclareLaunchArgument(
            'start_foxglove',
            default_value='true',
            description='Start foxglove_bridge on port 8765.'
        ),
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        clock_bridge,
        TimerAction(period=3.0, actions=[
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
        ]),
        twist_relay,
        foxglove_bridge,
    ])
