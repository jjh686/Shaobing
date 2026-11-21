"""Offline geometry and ROS gate regression checks. No serial port is opened."""
import math
import time
import unittest
from types import SimpleNamespace
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from rm_live_nav.geometry import (base_from_imu, frontier_goal, outside_footprint,
    quaternion_matrix, rpy_matrix, validate_mounting)
from rm_live_nav.motion_gate import MotionGate


def mounting():
    return dict(configured=True, fixed_gimbal=True, footprint_confirmed=True,
        parent_frame='base_footprint', child_frame='front_mid360',
        translation_m=[-.044, -.04425, .39336], rotation_rpy_rad=[0., math.pi/4, math.pi/2],
        footprint_xy_m=[[.29666, .46458], [-.23234, .46458], [-.23234, -.18172], [.29666, -.18172]])


class GeometryTests(unittest.TestCase):
    def test_unconfirmed_or_missing_mount_is_rejected(self):
        for field, value in [('configured', False), ('footprint_confirmed', False),
                             ('fixed_gimbal', False), ('translation_m', [None, 0., .39336]),
                             ('rotation_rpy_rad', [0., 0., float('nan')])]:
            data = mounting()
            data[field] = value
            with self.assertRaises(ValueError):
                validate_mounting(data)

    def test_reversed_polygon_is_rejected(self):
        data = mounting()
        data['footprint_xy_m'].reverse()
        with self.assertRaises(ValueError):
            validate_mounting(data)

    def test_tilted_lidar_and_imu_lever_arm_recover_chassis_pose(self):
        translation, rpy, _ = validate_mounting(mounting())
        base_lidar = rpy_matrix(rpy)
        imu_lidar_t = np.array([-.011, -.02329, .04412])
        imu_lidar_r = rpy_matrix([.1, -.2, .3])
        actual_base_r = rpy_matrix([0., 0., .7])
        actual_base_t = np.array([1., 2., .0])
        base_imu_r = base_lidar @ imu_lidar_r.T
        base_imu_t = translation - base_imu_r @ imu_lidar_t
        imu_r = actual_base_r @ base_imu_r
        imu_t = actual_base_t + actual_base_r @ base_imu_t
        recovered_r, recovered_t = base_from_imu(imu_r, imu_t, translation, base_lidar, imu_lidar_t, imu_lidar_r)
        np.testing.assert_allclose(recovered_r, actual_base_r, atol=1e-12)
        np.testing.assert_allclose(recovered_t, actual_base_t, atol=1e-12)

    def test_vehicle_points_removed_but_external_obstacle_retained(self):
        _, _, polygon = validate_mounting(mounting())
        self.assertEqual(outside_footprint(np.array([[0., 0.], [.2, .4], [.8, 0.], [0., -.8]]), polygon).tolist(), [False, False, True, True])

    def test_goal_uses_known_free_space_and_respects_rotated_map(self):
        cells = np.full((100, 100), -1)
        cells[10:90, 10:90] = 0
        origin = np.array([3., -5.])
        robot = np.array([5.05, 5.05]) @ rpy_matrix([0., 0., .7])[:2, :2].T + origin
        goal = frontier_goal(cells, .1, origin, .7, robot, .65, robot, 4.)
        self.assertIsNotNone(goal)
        local = (goal[0]-origin) @ rpy_matrix([0., 0., .7])[:2, :2]/.1
        self.assertEqual(cells[int(local[1]), int(local[0])], 0)
        self.assertLessEqual(np.linalg.norm(goal[0]-robot), 4.)

    def test_wall_blocks_unreachable_frontiers(self):
        cells = np.full((100, 100), 100)
        cells[10:90, 10:50] = 0  # sealed room with no unknown adjacency
        cells[:, 55:] = -1
        cells[10:90, 60:90] = 0
        robot = np.array([3., 5.])
        self.assertIsNone(frontier_goal(cells, .1, np.zeros(2), 0., robot, .65, robot, 20.))

    def test_too_narrow_and_blacklisted_frontiers_stop(self):
        cells = np.full((30, 30), -1)
        cells[13:17, :] = 0
        robot = np.array([1.5, 1.5])
        self.assertIsNone(frontier_goal(cells, .1, np.zeros(2), 0., robot, .65, robot, 3.))
        cells = np.full((100, 100), -1)
        cells[10:90, 10:90] = 0
        robot = np.array([5., 5.])
        self.assertIsNone(frontier_goal(cells, .1, np.zeros(2), 0., robot, .65, robot, .2))


class GateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.gate = MotionGate()
        self.populate()

    def tearDown(self):
        self.gate.destroy_node()

    def populate(self):
        stamp = self.gate.get_clock().now().to_msg()
        raw = PointCloud2()
        raw.header.stamp, raw.width, raw.height = stamp, 10, 1
        self.gate.mark('raw_cloud', raw)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'camera_init', 'base_footprint'
        odom.pose.pose.orientation.w = 1.
        self.gate.odom(odom)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'base_footprint'
        scan.range_min, scan.ranges = .1, [2.]*720
        self.gate.scan(scan)
        grid = OccupancyGrid()
        grid.header.stamp, grid.header.frame_id, grid.data = stamp, 'map', [0]*100
        self.gate.mark('map', grid)
        diagnostic = DiagnosticStatus(name='/chassis_comm', level=b'\x00',
            values=[KeyValue(key='fault_mask', value='0'), KeyValue(key='enabled', value='true')])
        self.gate.diagnose(DiagnosticArray(status=[diagnostic]))

    def arm(self):
        result = self.gate.enable(SetBool.Request(data=True), SetBool.Response())
        self.assertTrue(result.success, result.message)

    def test_defaults_stop_and_healthy_inputs_do_not_auto_arm(self):
        self.assertFalse(self.gate.armed)
        self.assertEqual(self.gate.health(), '')
        self.gate.tick()
        self.assertFalse(self.gate.armed)

    def test_stale_cloud_latches_off_until_manual_enable(self):
        self.arm()
        self.gate.received['raw_cloud'] -= 1.
        self.gate.tick()
        self.assertFalse(self.gate.armed)
        self.populate()
        self.gate.tick()
        self.assertFalse(self.gate.armed)

    def test_fault_and_obstacle_latch_off(self):
        self.arm()
        self.gate.diagnostic.values[0].value = '1'
        self.gate.tick()
        self.assertFalse(self.gate.armed)
        self.populate()
        self.arm()
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = self.gate.get_clock().now().to_msg(), 'base_footprint'
        scan.range_min, scan.ranges = .1, [.5]*720
        self.gate.scan(scan)
        self.gate.tick()
        self.assertFalse(self.gate.armed)

    def test_timestamp_replay_cannot_refresh_sensor_watchdog(self):
        self.arm()
        replay = PointCloud2()
        stamp = self.gate.source_stamps['raw_cloud']
        replay.header.stamp.sec, replay.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        self.gate.received['raw_cloud'] -= 1.
        self.assertFalse(self.gate.mark('raw_cloud', replay))
        self.gate.tick()
        self.assertFalse(self.gate.armed)

    def test_invalid_orientation_and_wrong_frame_cannot_arm(self):
        self.gate.level_pose = False
        result = self.gate.enable(SetBool.Request(data=True), SetBool.Response())
        self.assertFalse(result.success)
        with self.assertRaises(ValueError):
            quaternion_matrix([0., 0., 0., 0.])

    def test_three_metre_limit_stops(self):
        self.arm()
        self.gate.position = np.array([3.1, 0.])
        self.gate.tick()
        self.assertFalse(self.gate.armed)

    def test_explorer_heartbeat_loss_requires_manual_rearming(self):
        self.arm()
        self.gate.activity = (True, time.monotonic()-1.)
        self.gate.tick()
        self.assertFalse(self.gate.armed)
        self.gate.active(Bool(data=True))
        self.gate.tick()
        self.assertFalse(self.gate.armed)

    def test_activity_required_and_output_speed_is_limited(self):
        outputs = []
        self.gate.output = SimpleNamespace(publish=outputs.append)
        self.arm()
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = 1., 1., 2.
        self.gate.cmd(command)
        self.gate.tick()
        self.assertEqual(outputs[-1].twist.linear.x, 0.)
        self.gate.active(Bool(data=True))
        self.gate.cmd(command)
        self.gate.tick()
        out = outputs[-1]
        self.assertAlmostEqual(math.hypot(out.twist.linear.x, out.twist.linear.y), .1)
        self.assertEqual(out.twist.angular.z, .25)
        original_stamp = out.header.stamp
        self.gate.tick()
        self.assertEqual(outputs[-1].header.stamp, original_stamp)
        self.gate.command = (command, time.monotonic()-1., original_stamp)
        self.gate.tick()
        self.assertEqual(outputs[-1].twist.linear.x, 0.)


if __name__ == '__main__':
    unittest.main(verbosity=2)
