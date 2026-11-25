"""Standalone ABB planning demo. Do not launch the IK/collision demo alongside it."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def nodes(context):
    share = Path(get_package_share_directory('roscon_planner'))
    urdf = Path(LaunchConfiguration('urdf_file').perform(context)).expanduser().resolve()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(urdf.read_text(), value_type=str)}]),
        Node(package='roscon_planner', executable='planner_node', output='screen',
             parameters=[LaunchConfiguration('parameters').perform(context),
                         {'urdf_file': str(urdf),
                          'environment_file': LaunchConfiguration('environment_file').perform(context)}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'config/demo.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('roscon_planner'))
    robot = Path(get_package_share_directory('roscon_description'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('urdf_file', default_value=str(robot / 'urdf/irb2600.urdf')),
        DeclareLaunchArgument('environment_file', default_value=str(share / 'config/environment.yaml')),
        DeclareLaunchArgument('parameters', default_value=str(share / 'config/planner.yaml')),
        OpaqueFunction(function=nodes),
    ])
