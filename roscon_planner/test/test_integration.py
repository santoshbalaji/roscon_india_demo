"""Plan, switch collision methods, validate, preview and reject blocked requests."""
import copy
import os
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from roscon_msgs.srv import PlanMotion, SetCollisionBackend, ValidateTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def test_planner():
    process = subprocess.Popen([
        os.environ['PLANNER_EXECUTABLE'], '--ros-args',
        '-p', 'urdf_file:=' + os.environ['PLANNER_URDF'],
        '-p', 'environment_file:=' + os.environ['PLANNER_ENVIRONMENT'],
        '-p', 'stomp_iterations:=5', '-p', 'stomp_rollouts:=4',
    ])
    rclpy.init()
    node = rclpy.create_node('planner_integration_test')
    received = {}
    latched = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(PoseStamped, 'planner/demo_target', lambda msg: received.update(demo=msg), latched),
        node.create_subscription(JointState, 'joint_states', lambda msg: received.update(joints=msg), 10),
        node.create_subscription(JointTrajectory, 'planner/trajectory', lambda msg: received.update(trajectory=msg), latched),
    ]
    clients = {
        'plan': node.create_client(PlanMotion, 'planner/plan'),
        'backend': node.create_client(SetCollisionBackend, 'planner/set_backend'),
        'validate': node.create_client(ValidateTrajectory, 'planner/validate'),
        'play': node.create_client(Trigger, 'planner/play'),
        'stop': node.create_client(Trigger, 'planner/stop'),
        'reset': node.create_client(Trigger, 'planner/reset'),
    }

    def wait(predicate, seconds=60):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert process.poll() is None, 'Planner exited unexpectedly'
            rclpy.spin_once(node, timeout_sec=0.03)
            if predicate():
                return
        raise AssertionError('Planner test timed out')

    def call(name, request):
        future = clients[name].call_async(request)
        wait(future.done)
        return future.result()

    try:
        wait(lambda: 'demo' in received and 'joints' in received and all(c.service_is_ready() for c in clients.values()))
        initial = list(received['joints'].position)
        request = PlanMotion.Request()
        request.target = received['demo']
        request.ompl_planner = "rrt_connect"
        request.optimizer = "stomp"
        result = call('plan', request)
        assert result.success, result.message
        assert len(result.trajectory.points) >= 2
        assert list(received['joints'].position) == initial
        for backend in ('spheres', 'sdf', 'fcl'):
            change = SetCollisionBackend.Request()
            change.backend = backend
            assert call('backend', change).success
            assert not call('play', Trigger.Request()).success
            checked = call('validate', ValidateTrajectory.Request())
            if checked.valid:
                assert checked.checked_states >= len(result.trajectory.points)
            else:
                # Conservative spheres may reject a mesh-clear FCL path.
                assert not call('play', Trigger.Request()).success
                result = call('plan', request)
                assert result.success, result.message
                assert call('validate', ValidateTrajectory.Request()).valid
        bad = SetCollisionBackend.Request()
        bad.backend = 'unknown'
        assert not call('backend', bad).success
        assert call('play', Trigger.Request()).success
        wait(lambda: max(abs(a-b) for a, b in zip(initial, received['joints'].position)) > 0.02)
        assert call('stop', Trigger.Request()).success
        # Drain any joint-state sample already queued before the stop response.
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.03)
        paused = list(received['joints'].position)
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.03)
        assert list(received['joints'].position) == paused
        assert call('play', Trigger.Request()).success
        endpoint = result.trajectory.points[-1].positions
        wait(lambda: max(abs(a-b) for a, b in zip(endpoint, received['joints'].position)) < 1e-6)
        assert call('reset', Trigger.Request()).success
        malformed = ValidateTrajectory.Request()
        malformed.trajectory.joint_names = ['wrong_joint']
        malformed.trajectory.points = [JointTrajectoryPoint(positions=[0.0]), JointTrajectoryPoint(positions=[1.0])]
        assert not call('validate', malformed).valid
        blocked = copy.deepcopy(request)
        blocked.target.pose.position.x = 0.65
        blocked.target.pose.position.y = -0.65
        blocked.target.pose.position.z = 1.0
        rejected = call('plan', blocked)
        assert not rejected.success
        assert not rejected.trajectory.points
        assert not call('play', Trigger.Request()).success
        wait(lambda: 'trajectory' in received and not received['trajectory'].points)
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
