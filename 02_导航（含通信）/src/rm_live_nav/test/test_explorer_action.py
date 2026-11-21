"""Exercise the exploration action and cancellation over DDS, without motion."""
import json
import os
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster
from rm_live_nav.frontier_explorer import FrontierExplorer


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '97' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    explorer = FrontierExplorer()
    harness = rclpy.create_node('explorer_action_harness')
    executor = SingleThreadedExecutor()
    executor.add_node(explorer)
    executor.add_node(harness)
    broadcaster = TransformBroadcaster(harness)
    goals, cancellations = [], []
    result_waiter = Future()
    async def execute(handle):
        goals.append(handle)
        await result_waiter
        if handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.succeed()
        return NavigateToPose.Result()
    def cancelled(handle):
        cancellations.append(handle)
        return CancelResponse.ACCEPT
    server = ActionServer(harness, NavigateToPose, 'navigate_to_pose', execute, cancel_callback=cancelled)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 80
    grid.info.resolution = .1
    grid.info.origin.position.x = grid.info.origin.position.y = -4.
    grid.info.origin.orientation.w = 1.
    cells = np.full((80, 80), -1, dtype=np.int8)
    cells[15:65, 15:65] = 0
    grid.data = cells.ravel().tolist()
    def pump(seconds, ready=True):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = harness.get_clock().now().to_msg(), 'map', 'base_footprint'
            tf.transform.rotation.w = 1.
            broadcaster.sendTransform(tf)
            grid.header.stamp = harness.get_clock().now().to_msg()
            explorer.on_map(grid)
            explorer.on_ready(Bool(data=ready))
            if cancellations and not result_waiter.done():
                result_waiter.set_result(None)
            executor.spin_once(timeout_sec=.01)
    report = {'synthetic_only': True, 'physical_usb_opened': False}
    try:
        pump(.5)
        assert not goals, 'Explorer sent a goal before explicit enable'
        enabled = explorer.enable(SetBool.Request(data=True), SetBool.Response())
        assert enabled.success, enabled.message
        pump(2.)
        assert len(goals) == 1, 'Expected one active NavigateToPose goal'
        pose = goals[0].request.pose.pose
        point = np.array([pose.position.x, pose.position.y])
        assert .5 <= np.linalg.norm(point) <= 3.
        assert abs(point[0]) <= 1.85 and abs(point[1]) <= 1.85, 'Goal does not leave footprint clearance from unknown'
        pump(.5, ready=False)
        assert not explorer.enabled and cancellations, 'Readiness loss must cancel action and stop exploration'
        pump(.5, ready=True)
        assert not explorer.enabled and len(goals) == 1, 'Restored readiness must not auto restart'
        report.update(passed=True, goals=len(goals), cancellations=len(cancellations),
                      goal_xy=point.tolist(), no_automatic_start=True, no_automatic_restart=True)
    finally:
        server.destroy()
        executor.shutdown()
        explorer.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()
        (Path(os.environ['RM_NAV_ROOT'])/'local/live-explorer-action-test-20261121.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
