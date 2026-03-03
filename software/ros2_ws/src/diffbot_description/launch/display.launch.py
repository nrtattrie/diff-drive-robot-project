import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


pkg_share = get_package_share_directory('diffbot_description')

default_urdf_path = os.path.join(pkg_share, 'urdf', 'diffbot.urdf.xacro')
default_rviz_path = os.path.join(pkg_share, 'rviz', 'diffbot.rviz')


def generate_launch_description():

    urdf_file_arg = DeclareLaunchArgument(
        name='urdf_file',
        default_value=default_urdf_path,
        description='Absolute path to the robot URDF/xacro file'
    )

    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_path,
        description='Absolute path to the RViz config file'
    )

    use_gui_arg = DeclareLaunchArgument(
        name='use_gui',
        default_value='true',
        description='Set to false to disable joint_state_publisher_gui'
    )

    use_rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Set to false to skip launching RViz2 (e.g. when using Foxglove)'
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', LaunchConfiguration('urdf_file')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # GUI version: interactive sliders for joint positions
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )

    # Non-GUI version: needed when use_gui:=false (e.g. Foxglove headless sessions)
    # Publishes zero joint states so robot_state_publisher can broadcast wheel transforms
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_gui'))
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        urdf_file_arg,
        rviz_config_arg,
        use_gui_arg,
        use_rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz2_node,
    ])
