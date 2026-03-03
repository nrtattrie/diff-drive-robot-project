import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = FindPackageShare('diffbot_description')

    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([pkg_share, 'urdf', 'diffbot.urdf.xacro'])
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}]
    )

    # Headless Gazebo — no display needed (EGL rendering on Pi 5)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '--headless-rendering', '-r', 'empty.sdf'],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'diffbot',
                   '-topic', 'robot_description'],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diffbot_base_controller'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        # Load controllers after robot is spawned
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_robot,
                          on_exit=[load_joint_state_broadcaster])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=load_joint_state_broadcaster,
                          on_exit=[load_base_controller])
        ),
    ])
