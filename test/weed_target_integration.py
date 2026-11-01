"""Exercise the real mission node with deterministic clock, TF and sensor data."""

from contextlib import contextmanager
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time

# Keep the test separate from any running simulation. Set before DDS starts.
os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 80)

import rclpy
from geometry_msgs.msg import PointStamped, Pose, PoseArray, TransformStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from rs1.msg import WombatStatus, WombatWeedGoal
from rs1.srv import WombatNextGoal
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray


@contextmanager
def running_mission(binary, settings, log):
    process = subprocess.Popen([
        binary, '--ros-args', '-p', 'use_sim_time:=true',
        '--params-file', settings,
        '-p', 'camera_width_of_view:=3.0',
        '-r', '/tf:=/husky1/tf', '-r', '/tf_static:=/husky1/tf_static',
    ], stdout=log, stderr=subprocess.STDOUT)
    try:
        yield process
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main():
    with tempfile.TemporaryDirectory(prefix='rs1-weed-integration-') as directory:
        os.environ['ROS_LOG_DIR'] = directory
        log_path = Path(directory) / 'mission.log'
        with log_path.open('w+') as log:
            with running_mission(sys.argv[1], sys.argv[2], log) as process:
                rclpy.init()
                node = rclpy.create_node('weed_target_integration')
                points, markers, late_markers = [], [], []
                weed_goals, statuses = [], []
                estimate_arrays, late_estimate_arrays = [], []
                latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                point_sub = node.create_subscription(PointStamped, '/husky1/target_weed', points.append, 5)
                marker_sub = node.create_subscription(Marker, '/husky1/target_weed_marker', markers.append, latched)
                estimates_sub = node.create_subscription(
                    MarkerArray, '/husky1/weed_estimate_markers', estimate_arrays.append, latched)
                node.create_subscription(WombatWeedGoal, '/wombat/weed_goal', weed_goals.append, 10)
                node.create_subscription(WombatStatus, '/wombat/status', statuses.append, latched)
                clock_pub = node.create_publisher(Clock, '/clock', 10)
                tf_pub = node.create_publisher(TFMessage, '/husky1/tf_static', latched)
                region_pub = node.create_publisher(PoseArray, '/region', 5)
                # Match the actual Gazebo bridge's reliable cloud publisher.
                depth_pub = node.create_publisher(PointCloud2, '/husky1/camera/depth/points', 10)
                from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
                detection_pub = node.create_publisher(Detection2DArray, '/husky1/detections', 5)
                client = node.create_client(WombatNextGoal, '/wombat_next_goal')
                simulated_time = 10

                def wait_until(predicate, timeout=8):
                    deadline = time.monotonic() + timeout
                    while time.monotonic() < deadline:
                        assert process.poll() is None, 'Mission node exited unexpectedly'
                        clock = Clock(); clock.clock.sec = simulated_time
                        clock_pub.publish(clock)
                        rclpy.spin_once(node, timeout_sec=0.03)
                        if predicate():
                            return
                    raise AssertionError('Timed out waiting for mission node output')

                def estimate_adds():
                    return [m for m in estimate_arrays[-1].markers if m.action == Marker.ADD] if estimate_arrays else []

                def send_observation(stamp, forward_depth, *, lateral=-0.7,
                                     publish_depth=True, publish_detections=True):
                    cloud = PointCloud2()
                    cloud.header.frame_id = 'husky1_camera_link'; cloud.header.stamp.sec = stamp
                    cloud.height = cloud.width = 21
                    cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                                    for i, name in enumerate(('x', 'y', 'z'))]
                    cloud.point_step = 12; cloud.row_step = 21 * 12
                    cloud.data = struct.pack('<fff', forward_depth, lateral, 0.15) * (21 * 21)
                    detections = Detection2DArray(); detections.header = cloud.header
                    detection = Detection2D(); detection.header = cloud.header
                    detection.bbox.center.position.x = detection.bbox.center.position.y = 10.0
                    detection.bbox.size_x = detection.bbox.size_y = 20.0
                    result = ObjectHypothesisWithPose()
                    result.hypothesis.class_id = 'Weed'; result.hypothesis.score = 0.9
                    detection.results = [result]; detections.detections = [detection]
                    if publish_depth:
                        depth_pub.publish(cloud)
                    if publish_detections:
                        detection_pub.publish(detections)

                try:
                    wait_until(lambda: client.service_is_ready() and region_pub.get_subscription_count() > 0
                               and tf_pub.get_subscription_count() > 0 and detection_pub.get_subscription_count() > 0
                               and depth_pub.get_subscription_count() > 0)
                    base_tf = TransformStamped()
                    base_tf.header.frame_id = 'map'; base_tf.child_frame_id = 'husky1_base_link'
                    base_tf.transform.translation.y = 1.0
                    base_tf.transform.rotation.z = math.sin(math.pi / 4)
                    base_tf.transform.rotation.w = math.cos(math.pi / 4)
                    camera_tf = TransformStamped()
                    camera_tf.header.frame_id = 'husky1_base_link'; camera_tf.child_frame_id = 'husky1_camera_link'
                    camera_tf.transform.translation.x = 0.45; camera_tf.transform.translation.z = 0.25
                    camera_tf.transform.rotation.w = 1.0
                    tf_pub.publish(TFMessage(transforms=[base_tf, camera_tf]))
                    region = PoseArray(); region.header.frame_id = 'map'
                    for x, y in ((0., 0.), (5., 0.), (5., 5.), (0., 5.)):
                        pose = Pose(); pose.position.x = x; pose.position.y = y; pose.orientation.w = 1.
                        region.poses.append(pose)
                    region_pub.publish(region)
                    wait_until(lambda: statuses and statuses[-1].region_reset and
                               statuses[-1].mission_generation > 0)
                    # Let newer detections arrive while older depth is still in
                    # flight. A latest-only pending slot would lose the first
                    # detection and fail to publish when its cloud finally arrives.
                    send_observation(10, 1.55, publish_depth=False)
                    send_observation(11, 1.75, publish_depth=False)
                    simulated_time = 11
                    callbacks_settled = time.monotonic() + 0.4
                    wait_until(lambda: time.monotonic() >= callbacks_settled)
                    send_observation(10, 1.55, publish_detections=False)
                    wait_until(lambda: len(estimate_adds()) == 1)
                    assert not points and not any(m.action == Marker.ADD for m in markers), \
                        'Estimation should be visible without selecting a target before a lane is active'
                    assert abs(estimate_adds()[0].pose.position.x - 0.7) < 1e-6

                    # Activate the first lane through the normal service. The weed
                    # is 0.7 m off its centre: valid for 3.0/2/2 = 0.75 m, but
                    # rejected by both the old 0.5 m and default-derived 0.625 m.
                    response = None
                    for _ in range(20):
                        future = client.call_async(WombatNextGoal.Request())
                        wait_until(future.done)
                        response = future.result()
                        if response.success:
                            break
                    assert response and response.success and not response.is_weed
                    assert response.goal.pose.position.x == 0 and response.goal.pose.position.y == 0
                    wait_until(lambda: points and any(m.action == Marker.ADD for m in markers))
                    wait_until(lambda: weed_goals)
                    assert weed_goals[-1].weed_id > 0 and weed_goals[-1].mission_generation == statuses[-1].mission_generation
                    assert weed_goals[-1].goal.header.frame_id == 'map'
                    assert abs(weed_goals[-1].goal.pose.orientation.z - math.sqrt(0.5)) < 1e-6
                    assert abs(weed_goals[-1].goal.pose.orientation.w - math.sqrt(0.5)) < 1e-6
                    estimate = points[-1]
                    assert estimate.header.frame_id == 'map' and estimate.header.stamp.sec == 10
                    assert abs(estimate.point.x - 0.7) < 1e-6
                    assert abs(estimate.point.y - 3.0) < 1e-6
                    assert abs(estimate.point.z - 0.4) < 1e-6
                    marker = next(m for m in reversed(markers) if m.action == Marker.ADD)
                    assert marker.pose.position == estimate.point and marker.id == 0
                    assert marker.color.r == 1 and marker.color.b == 0
                    assert marker.color.a == 1 and marker.lifetime.sec > 0

                    send_observation(11, 1.75, publish_detections=False)
                    wait_until(lambda: points[-1].header.stamp.sec == 11)
                    assert abs(points[-1].point.y - 3.1) < 1e-6, 'Matched position was not smoothed'

                    # A second valid estimate belongs to a different lane. It must
                    # be visible without replacing the eligible yellow target.
                    simulated_time = 13
                    send_observation(13, 1.55, lateral=-2.0)
                    wait_until(lambda: len(estimate_adds()) == 2)
                    assert points[-1].header.stamp.sec == 11
                    for blue in estimate_adds():
                        assert blue.header.frame_id == 'map' and blue.color.b == 1
                        assert blue.color.r == 0 and blue.scale.x < marker.scale.x
                        assert blue.lifetime.sec > 0
                    simulated_time = 15
                    wait_until(lambda: any(m.action == Marker.DELETE for m in markers) and len(estimate_adds()) == 1)
                    assert markers[-1].action == Marker.DELETE and markers[-1].id == 0
                    assert abs(estimate_adds()[0].pose.position.x - 2.0) < 1e-6
                    assert estimate_arrays[-1].markers[0].action == Marker.DELETEALL
                    simulated_time = 17
                    wait_until(lambda: estimate_arrays and not estimate_adds())
                    late_sub = node.create_subscription(
                        Marker, '/husky1/target_weed_marker', late_markers.append, latched)
                    late_estimates_sub = node.create_subscription(
                        MarkerArray, '/husky1/weed_estimate_markers', late_estimate_arrays.append, latched)
                    wait_until(lambda: late_markers and late_estimate_arrays)
                    assert late_markers[-1].action == Marker.DELETE, 'Late RViz subscriber would see a stale marker'
                    assert [m.action for m in late_estimate_arrays[-1].markers] == [Marker.DELETEALL], \
                        'Late RViz subscriber would see expired estimates'
                    # The same configured width must also determine actual
                    # waypoint spacing, rather than just the target corridor.
                    request = WombatNextGoal.Request()
                    request.operation = WombatNextGoal.Request.SKIP_CURRENT_GOAL
                    future = client.call_async(request)
                    wait_until(future.done)
                    response = future.result()
                    assert response.success and not response.is_weed
                    assert response.goal.pose.position.x == 0.0
                    assert response.goal.pose.position.y == 3.0
                    print('PASS: real mission node publishes accurate map coordinates, updates the target, '
                          'derives corridor width from configured spacing, displays off-lane estimates, '
                          'keeps search goals, and clears expired markers for existing and late subscribers')
                except Exception:
                    log.flush()
                    print(log_path.read_text(), file=sys.stderr)
                    raise
                finally:
                    node.destroy_node()
                    rclpy.shutdown()



if __name__ == '__main__':
    main()
