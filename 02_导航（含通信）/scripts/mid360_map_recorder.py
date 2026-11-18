#!/usr/bin/env python3
"""Accumulate registered scans into a voxel PCD and save the current ROS grid."""

import json
import math
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
import yaml


class MapRecorder(Node):
    def __init__(self):
        super().__init__("mid360_test_recorder")
        self.output = Path(self.declare_parameter("output_dir", "").value)
        self.voxel_size = self.declare_parameter("voxel_size", 0.10).value
        self.max_voxels = self.declare_parameter("max_voxels", 1000000).value
        self.voxels = {}
        self.grid = None
        self.header = Header(frame_id="camera_init")
        self.frames = 0
        self.limit_reported = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(PointCloud2, "mid360_test/map_cloud", latched)
        self.create_subscription(PointCloud2, "cloud_registered", self.on_cloud, 5)
        self.create_subscription(OccupancyGrid, "map", self.on_grid, latched)
        self.create_service(Trigger, "mid360_test/save_map", self.on_save)
        self.create_timer(2.0, self.publish_cloud)
        self.create_timer(60.0, self.autosave)
        self.get_logger().info(f"Map output: {self.output}")

    def on_cloud(self, msg):
        if msg.header.frame_id != "camera_init":
            self.get_logger().error("Expected registered cloud in camera_init")
            return
        points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"),
                                               skip_nans=True)
        points = points[np.isfinite(points).all(axis=1)]
        if not len(points):
            return
        keys = np.floor(points / self.voxel_size).astype(np.int64)
        _, indices = np.unique(keys, axis=0, return_index=True)
        for index in indices:
            key = tuple(keys[index])
            if key in self.voxels or len(self.voxels) < self.max_voxels:
                self.voxels[key] = points[index].copy()
            elif not self.limit_reported:
                self.get_logger().warning("Voxel limit reached; save and restart for a larger area")
                self.limit_reported = True
        self.header = msg.header
        self.frames += 1

    def on_grid(self, msg):
        self.grid = msg

    def cloud_array(self):
        return np.asarray(list(self.voxels.values()), dtype="<f4").reshape(-1, 3)

    def publish_cloud(self):
        if self.voxels:
            self.publisher.publish(point_cloud2.create_cloud_xyz32(self.header, self.cloud_array()))

    @staticmethod
    def write_atomic(path, data):
        temporary = path.with_name(path.name + ".tmp")
        temporary.write_bytes(data)
        temporary.replace(path)

    def save(self):
        if not self.voxels:
            raise RuntimeError("No registered point cloud yet; check lidar and Point-LIO")
        self.output.mkdir(parents=True, exist_ok=True)
        points = self.cloud_array()
        count = len(points)
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
            "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            f"WIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {count}\nDATA binary\n"
        )
        self.write_atomic(self.output / "map.pcd", header.encode("ascii") + points.tobytes())
        summary = {"point_count": count, "voxel_size_m": self.voxel_size,
                   "registered_frames": self.frames, "cloud_frame": self.header.frame_id,
                   "bounds_m": {"min": points.min(axis=0).tolist(),
                                "max": points.max(axis=0).tolist()},
                   "voxel_limit_reached": self.limit_reported,
                   "occupancy_grid_saved": self.grid is not None}
        if self.grid is not None:
            grid = self.grid
            cells = np.asarray(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)
            pixels = np.full(cells.shape, 205, dtype=np.uint8)
            pixels[(cells >= 0) & (cells <= 25)] = 254
            pixels[cells >= 65] = 0
            image = f"P5\n{grid.info.width} {grid.info.height}\n255\n".encode("ascii")
            self.write_atomic(self.output / "map.pgm", image + np.flipud(pixels).tobytes())
            q = grid.info.origin.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            metadata = {"image": "map.pgm", "mode": "trinary",
                        "resolution": grid.info.resolution,
                        "origin": [grid.info.origin.position.x, grid.info.origin.position.y, yaw],
                        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.25}
            self.write_atomic(self.output / "map.yaml", yaml.safe_dump(metadata).encode("utf-8"))
            summary["grid"] = {"width": grid.info.width, "height": grid.info.height,
                               "resolution_m": grid.info.resolution,
                               "known_cells": int((cells >= 0).sum()),
                               "occupied_cells": int((cells >= 65).sum())}
        self.write_atomic(self.output / "summary.json",
                          json.dumps(summary, indent=2).encode("utf-8"))
        return summary

    def on_save(self, request, response):
        del request
        try:
            result = self.save()
            response.success = result["occupancy_grid_saved"]
            response.message = (f"Saved {result['point_count']} points to {self.output}; "
                                f"2D map saved: {result['occupancy_grid_saved']}")
        except Exception as error:
            response.success = False
            response.message = str(error)
        self.get_logger().info(response.message)
        return response

    def autosave(self):
        if self.voxels:
            try:
                self.save()
            except Exception as error:
                self.get_logger().error(f"Map save failed: {error}")


def main():
    rclpy.init()
    node = MapRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.autosave()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
