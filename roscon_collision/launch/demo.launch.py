"""Start the ABB collision demo, or load a different expanded URDF and scene."""
from pathlib import Path
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def create_nodes(context):
    robot_file = LaunchConfiguration('robot_file').perform(context)
    robot_description = Path(robot_file).read_text()
    robot_xml = ElementTree.fromstring(robot_description)
    child_links = {joint.find('child').attrib['link'] for joint in robot_xml.findall('joint')}
    root_links = [link.attrib['name'] for link in robot_xml.findall('link')
                  if link.attrib['name'] not in child_links]
    if len(root_links) != 1:
        raise ValueError('The URDF must have exactly one root link')
    description_parameter = {'robot_description': ParameterValue(robot_description, value_type=str)}
    collision_share = get_package_share_directory('roscon_collision')
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    use_panel_controls = LaunchConfiguration('gui').perform(context).lower() == 'true'
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[description_parameter]),
        Node(package='roscon_collision', executable='collision_node', output='screen',
             parameters=[description_parameter, {
                 'environment_file': LaunchConfiguration('environment_file'),
                 'update_rate': 20.0,
                 'collision_backend': LaunchConfiguration('collision_backend'),
                 'sphere_radius': ParameterValue(LaunchConfiguration('sphere_radius'), value_type=float),
                 'sdf_cell_size': ParameterValue(LaunchConfiguration('sdf_cell_size'), value_type=float),
             }]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(Path(collision_share) / 'rviz' / 'collision.rviz'),
                        '-f', root_links[0]],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]
    # Without panel controls, publish the default pose from the headless publisher.
    if not use_rviz or not use_panel_controls:
        nodes.append(Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            parameters=[description_parameter]))
    return nodes


def generate_launch_description():
    description_share = get_package_share_directory('roscon_description')
    collision_share = get_package_share_directory('roscon_collision')
    return LaunchDescription([
        DeclareLaunchArgument('robot_file',
                              default_value=str(Path(description_share) / 'urdf' / 'irb2600.urdf'),
                              description='Absolute path to an expanded robot URDF'),
        DeclareLaunchArgument('environment_file',
                              default_value=str(Path(collision_share) / 'config' / 'environment.yaml'),
                              description='Environment YAML path or package URI'),
        DeclareLaunchArgument('collision_backend', default_value='fcl', choices=['fcl', 'gpu_spheres', 'gpu_sdf']),
        DeclareLaunchArgument('sphere_radius', default_value='0.10'),
        DeclareLaunchArgument('sdf_cell_size', default_value='0.08'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Use the RViz panel for joint control; false uses the headless publisher'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=create_nodes),
    ])
