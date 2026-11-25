"""Exercise post-filtering through the real ROS node without moving the robot."""
import copy
import os
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from roscon_msgs.msg import IKSelection, IKSolutions
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray


def test_collision_filter():
    process = subprocess.Popen([
        os.environ['IK_EXECUTABLE'], '--ros-args',
        '--params-file', os.environ['IK_PARAMETERS'],
        '-p', 'urdf_file:=' + os.environ['IK_URDF'],
        '-p', 'environment_file:=' + os.environ['IK_ENVIRONMENT'],
    ])
    rclpy.init()
    node = rclpy.create_node('test_ik_collision_filter')
    received = {}
    latched = rclpy.qos.QoSProfile(
        depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(PoseStamped, 'ik_target_pose',
                                 lambda msg: received.update(target=msg), 10),
        node.create_subscription(IKSolutions, 'ik_solutions',
                                 lambda msg: received.update(result=msg), latched),
        node.create_subscription(MarkerArray, 'ik_environment',
                                 lambda msg: received.update(environment=msg), latched),
        node.create_subscription(JointState, 'joint_states',
                                 lambda msg: received.update(joints=msg), 10),
    ]
    search = node.create_publisher(PoseStamped, 'ik_find_solutions', 10)
    select = node.create_publisher(IKSelection, 'ik_select_solution', 10)

    def wait_for(predicate):
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            assert process.poll() is None, 'IK node exited unexpectedly'
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError('Timed out waiting for IK collision results')

    try:
        wait_for(lambda: len(received) == 4 and search.get_subscription_count() > 0
                 and select.get_subscription_count() > 0)
        assert len(received['environment'].markers) == 3
        original_target = copy.deepcopy(received['target'])
        original_joints = list(received['joints'].position)
        cases = [None, (0.65, -0.65, 1.0), (1.15, 0.0, 0.45), (0.65, 0.65, 0.9)]
        for xyz in cases:
            target = copy.deepcopy(original_target)
            if xyz is not None:
                target.pose.position.x, target.pose.position.y, target.pose.position.z = xyz
            generation = received['result'].generation
            search.publish(target)
            wait_for(lambda: received['result'].generation > generation
                     and received['result'].seeds_total == 17
                     and received['result'].complete)
            result = received['result']
            assert result.check_errors == 0
            assert list(received['joints'].position) == original_joints
            if xyz is None:
                assert len(result.solutions) >= 2
                valid_index = len(result.solutions) - 1
                alternative = result.solutions[valid_index].joints.position
                assert max(abs(a - b) for a, b in zip(alternative, original_joints)) > 0.1
                valid_generation = result.generation
            else:
                assert not result.solutions
                assert result.environment_collision_rejections > 0
                assert 'ENVIRONMENT:' in result.last_rejection
                if xyz == (0.65, -0.65, 1.0):
                    assert result.self_collision_rejections > 0
        # A delayed preview from the old, free target cannot apply after rejection.
        selection = IKSelection()
        selection.generation = valid_generation
        selection.index = valid_index
        select.publish(selection)
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert list(received['joints'].position) == original_joints
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
