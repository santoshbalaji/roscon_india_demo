"""Exercise the real C++ node through ROS topics with known collision poses."""
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import MarkerArray
import yaml


ROBOT = '''<robot name="test_robot">
  <link name="base_link">
    <collision><geometry><sphere radius="0.25"/></geometry></collision>
  </link>
  <link name="spacer"/>
  <joint name="slide" type="prismatic">
    <parent link="base_link"/><child link="spacer"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="4" effort="1" velocity="1"/>
  </joint>
  <link name="tip">
    <collision><geometry><sphere radius="0.25"/></geometry></collision>
  </link>
  <joint name="tip_mount" type="fixed">
    <parent link="spacer"/><child link="tip"/>
  </joint>
</robot>'''


def make_mesh(path):
    # Closed cube; an offset sphere intersects its boundary in the test.
    path.write_text('''v -0.25 -0.25 -0.25
v 0.25 -0.25 -0.25
v 0.25 0.25 -0.25
v -0.25 0.25 -0.25
v -0.25 -0.25 0.25
v 0.25 -0.25 0.25
v 0.25 0.25 0.25
v -0.25 0.25 0.25
f 1 3 2
f 1 4 3
f 5 6 7
f 5 7 8
f 1 2 6
f 1 6 5
f 2 3 7
f 2 7 6
f 3 4 8
f 3 8 7
f 4 1 5
f 4 5 8
''')


@pytest.mark.parametrize('backend', ['fcl', 'gpu_spheres', 'gpu_sdf'])
@pytest.mark.parametrize('shape_type', ['box', 'sphere', 'cylinder', 'mesh', 'both'])
def test_collision_changes_and_colors(tmp_path, shape_type, backend):
    if backend != 'fcl' and os.environ.get('ROSCON_TEST_CUDA') != '1':
        pytest.skip('Set ROSCON_TEST_CUDA=1 to run GPU ROS integration tests')
    obstacle = {'name': 'obstacle', 'position': [2.0, 0.0, 0.0]}
    if shape_type == 'box':
        obstacle.update(type='box', size=[0.5, 0.5, 0.5])
    elif shape_type == 'cylinder':
        obstacle.update(type='cylinder', radius=0.25, length=0.5)
    elif shape_type == 'mesh':
        mesh_path = tmp_path / 'cube.obj'
        make_mesh(mesh_path)
        obstacle.update(type='mesh', resource=str(mesh_path), scale=[1.0, 1.0, 1.0])
    else:
        obstacle.update(type='sphere', radius=0.25)
    if shape_type == 'both':
        obstacle['position'] = [0.0, 0.0, 0.0]
    environment_path = tmp_path / 'environment.yaml'
    environment_path.write_text(yaml.safe_dump({'objects': [obstacle]}))
    parameters_path = tmp_path / 'parameters.yaml'
    parameters_path.write_text(yaml.safe_dump({'roscon_collision': {'ros__parameters': {
        'robot_description': ROBOT,
        'environment_file': str(environment_path),
        'update_rate': 30.0,
        'collision_backend': backend,
    }}}))
    process_environment = os.environ.copy()
    process_environment['ROS_LOG_DIR'] = str(tmp_path / 'logs')
    log_file = (tmp_path / 'node.log').open('w')
    process = subprocess.Popen(
        [os.environ['COLLISION_EXECUTABLE'], '--ros-args', '--params-file', str(parameters_path)],
        stdout=log_file, stderr=subprocess.STDOUT, env=process_environment)
    rclpy.init()
    node = rclpy.create_node('collision_test')
    publisher = node.create_publisher(JointState, 'joint_states', 10)
    received = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, 'collision_status',
                             lambda message: received.update(status=message.data), qos)
    node.create_subscription(MarkerArray, 'collision_markers',
                             lambda message: received.update(markers=message.markers), qos)

    node.create_subscription(MarkerArray, 'sphere_markers',
                             lambda message: received.update(spheres=message.markers), qos)

    def call_service(service_type, name, request):
        client = node.create_client(service_type, name)
        assert client.wait_for_service(timeout_sec=5.0)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        assert future.done(), f'{name} timed out'
        response = future.result()
        assert response.success, response.message
        node.destroy_client(client)
        return response

    node.create_subscription(MarkerArray, 'sdf_markers',
                             lambda message: received.update(sdf=message.markers), qos)

    active_backend = backend

    def wait_for(position, expected_status, expected_color):
        received.clear()
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path / 'node.log').read_text()
            message = JointState()
            message.name = ['slide']
            message.position = [position]
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
            if active_backend != 'fcl':
                markers = received.get('spheres', [])
                inactive_markers = received.get('markers')
                namespace = 'gpu_spheres'
            else:
                markers = received.get('markers', [])
                inactive_markers = received.get('spheres')
                namespace = 'robot_collision'
            tip = next((marker for marker in markers
                        if marker.ns == namespace and marker.id == 1), None)
            if inactive_markers is None:
                continue
            visible_inactive = [marker for marker in inactive_markers if marker.action == marker.ADD]
            if active_backend == 'gpu_sdf':
                if not visible_inactive or any(marker.ns != 'environment' for marker in visible_inactive):
                    continue
            elif visible_inactive:
                continue
            if 'sdf' not in received:
                continue
            slices = [marker for marker in received['sdf'] if marker.action == marker.ADD]
            if active_backend == 'gpu_sdf':
                if not slices:
                    continue
                assert all(marker.type == marker.CUBE_LIST for marker in slices)
                for marker in slices:
                    assert len(marker.points) == len(marker.colors)
                    assert any(color.r > color.b for color in marker.colors)  # Inside.
                    assert any(color.b > color.r for color in marker.colors)  # Outside.
            elif slices:
                continue
            if (expected_status in received.get('status', '') and tip is not None
                    and abs(tip.pose.position.x - position) < 1e-6):
                color = (tip.color.r, tip.color.g, tip.color.b)
                if color == pytest.approx(expected_color):
                    return
        pytest.fail(f'Expected {expected_status} at {position}, received {received}')

    try:
        if shape_type == 'both':
            wait_for(0.0, 'SELF: base_link <-> tip', (1.0, 0.9, 0.0))
            assert 'ENVIRONMENT: tip <-> obstacle' in received['status']
        else:
            wait_for(0.0, 'SELF: base_link <-> tip', (1.0, 0.0, 1.0))
            wait_for(1.8, 'ENVIRONMENT: tip <-> obstacle', (1.0, 0.0, 0.0))
            if active_backend == 'gpu_spheres':
                obstacle_marker = next(marker for marker in received['spheres']
                                       if marker.ns == 'gpu_spheres' and marker.id >= 2)
            else:
                obstacle_marker = next(marker for marker in received['markers']
                                       if marker.ns == 'environment')
            assert obstacle_marker.color.r == 1.0
            assert obstacle_marker.color.g == 0.0
            wait_for(4.0, 'Collision free', (1.0, 0.45, 0.08))
        if backend != 'fcl':
            validation_service = '/roscon_collision/validate_spheres'
            if backend == 'gpu_sdf':
                validation_service = '/roscon_collision/validate_sdf'
            validation = call_service(Trigger, validation_service, Trigger.Request())
            assert '0 mismatches' in validation.message
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                spheres = [marker for marker in received.get('spheres', [])
                           if marker.action == marker.ADD]
                if spheres:
                    break
            assert spheres
            if backend == 'gpu_sdf':
                assert len(spheres) == 2  # Robot only; environment uses its SDF surfaces.
            assert all(marker.type == marker.SPHERE for marker in spheres)
            assert all(marker.scale.x > 0.0 for marker in spheres)
            # One backend selection must change both reporting and visualization.
            call_service(SetBool, '/roscon_collision/use_gpu_spheres', SetBool.Request(data=False))
            active_backend = 'fcl'
            if shape_type != 'both':
                wait_for(4.0, 'Collision free', (1.0, 0.45, 0.08))
                assert not any(marker.action == marker.ADD for marker in received.get('spheres', []))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log_file.close()
