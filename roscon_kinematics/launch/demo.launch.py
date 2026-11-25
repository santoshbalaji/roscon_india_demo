"""Launch the ROSCon ABB IRB 2600 description, constrained C++ IK, and RViz."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def create_nodes(context):
    share = Path(get_package_share_directory('roscon_kinematics'))
    robot_file = Path(LaunchConfiguration('urdf_file').perform(context)).expanduser().resolve()
    robot_description = robot_file.read_text()
    parameters = LaunchConfiguration('parameters').perform(context)
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}]),
        Node(package='roscon_kinematics', executable='ik_node', name='roscon_kinematics',
             output='screen', parameters=[parameters, {'urdf_file': str(robot_file),
                 'environment_file': LaunchConfiguration('environment_file').perform(context)}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'config/demo.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('roscon_kinematics'))
    description = Path(get_package_share_directory('roscon_description'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('urdf_file',
                              default_value=str(description / 'urdf/irb2600.urdf'),
                              description='Expanded URDF; defaults to the existing ABB model'),
        DeclareLaunchArgument('environment_file', default_value=str(share / 'config/environment.yaml')),
        DeclareLaunchArgument('parameters', default_value=str(share / 'config/solver.yaml')),
        OpaqueFunction(function=create_nodes),
    ])
