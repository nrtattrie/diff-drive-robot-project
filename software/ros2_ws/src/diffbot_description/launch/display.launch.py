
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node


# Launching the robot state publisher node
# The launch system imports your file and calls this function at launch time
def generate_launch_description():
    # Finding the package share in my directory
    pkg_share = get_package_share_directory('diffbot_description')

    # Processing xacro inside my launch file
    robot_description = Command([FindExecutable(name='xacro'), ' ', os.path.join(pkg_share, 'urdf', 'diffbot.urdf.xacro')])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        rsp_node,
        jsp_node
    ])
