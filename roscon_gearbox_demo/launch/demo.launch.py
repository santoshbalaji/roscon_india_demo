"""Launch one robot and the existing planner GUI in the gearbox cell."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def create_nodes(context):
    cell = Path(get_package_share_directory('roscon_gearbox_demo'))
    planner = Path(get_package_share_directory('roscon_planner'))
    description = Path(get_package_share_directory('roscon_description'))
    urdf = description / 'urdf/irb2600.urdf'
    presets = yaml.safe_load((cell / 'config/targets.yaml').read_text())
    target = LaunchConfiguration('target').perform(context)
    if target not in presets['targets']:
        raise ValueError('Unknown target. Choose: ' + ', '.join(presets['targets']))
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(urdf.read_text(), value_type=str)}]),
        Node(package='roscon_planner', executable='planner_node', output='screen',
             parameters=[str(planner / 'config/planner.yaml'), {
                 'urdf_file': str(urdf),
                 'environment_file': str(cell / 'config/environment.yaml'),
                 'initial_joints': presets['home'],
                 'demo_joints': presets['targets'][target],
                 'collision_backend': LaunchConfiguration('backend').perform(context),
             }]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(planner / 'config/demo.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('backend', default_value='fcl', choices=['fcl', 'spheres', 'sdf']),
        DeclareLaunchArgument('target', default_value='housing_inspection',
                              description='Preset loaded by the panel Load demo target button'),
        OpaqueFunction(function=create_nodes),
    ])
