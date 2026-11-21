"""Coordinate math and reachable frontier selection, independent of ROS."""
from collections import deque
import math
import numpy as np
from scipy import ndimage


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or abs(norm-1.) > 0.05:
        raise ValueError('Invalid orientation quaternion')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def matrix_quaternion(r):
    # scipy returns [x, y, z, w], matching ROS.
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(r).as_quat()


def validate_mounting(config):
    if not all(config.get(k) is True for k in ('configured', 'fixed_gimbal', 'footprint_confirmed')):
        raise ValueError('Confirm measured mounting, fixed yaw/pitch and protrusion-inclusive footprint in live_mounting.yaml')
    if config.get('parent_frame') != 'base_footprint' or config.get('child_frame') != 'front_mid360':
        raise ValueError('Expected base_footprint -> front_mid360')
    xyz = np.asarray(config['translation_m'], dtype=float)
    rpy = np.asarray(config['rotation_rpy_rad'], dtype=float)
    footprint = np.asarray(config['footprint_xy_m'], dtype=float)
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite(np.r_[xyz, rpy]).all():
        raise ValueError('Fill finite signed xyz and roll/pitch/yaw measurements')
    if footprint.ndim != 2 or footprint.shape[0] < 3 or footprint.shape[1] != 2 or not np.isfinite(footprint).all():
        raise ValueError('Invalid footprint polygon')
    # Require a convex CCW polygon containing the chassis origin.
    edges = np.roll(footprint, -1, axis=0)-footprint
    turns = np.cross(edges, np.roll(edges, -1, axis=0))
    sides = np.cross(edges, -footprint)
    if (turns <= 0).any() or (sides <= 0).any():
        raise ValueError('Footprint must be convex, counterclockwise and contain the chassis origin')
    return xyz, rpy, footprint


def base_from_imu(imu_r, imu_t, base_lidar_t, base_lidar_r, imu_lidar_t, imu_lidar_r):
    # Point-LIO extrinsic_R/T mean T_IMU_LIDAR (not T_LIDAR_IMU).
    base_imu_r = base_lidar_r @ imu_lidar_r.T
    base_imu_t = base_lidar_t - base_imu_r @ imu_lidar_t
    world_base_r = imu_r @ base_imu_r.T
    return world_base_r, imu_t - world_base_r @ base_imu_t


def outside_footprint(points_xy, footprint, padding=0.03):
    edges = np.roll(footprint, -1, axis=0)-footprint
    delta = points_xy[:, None, :]-footprint[None, :, :]
    cross = edges[None, :, 0]*delta[:, :, 1]-edges[None, :, 1]*delta[:, :, 0]
    inside = (cross >= -padding*np.linalg.norm(edges, axis=1)).all(axis=1)
    return ~inside


def frontier_goal(cells, resolution, origin_xy, origin_yaw, robot_xy, clearance,
                  start_xy, max_radius, blacklist=(), min_cluster=5):
    """Return a safe reachable free-space goal near a free/unknown boundary.

    Both obstacles and unknown cells are inflated by the circumscribed footprint.
    A four-connected flood fill prohibits crossing walls and diagonal corners.
    """
    cells = np.asarray(cells)
    if cells.ndim != 2 or cells.size > 4_000_000 or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Invalid/oversized occupancy grid')
    rot = rpy_matrix((0., 0., origin_yaw))[:2, :2]
    rc = (np.asarray(robot_xy)-origin_xy) @ rot / resolution
    col, row = np.floor(rc).astype(int)
    height, width = cells.shape
    if not (0 <= row < height and 0 <= col < width):
        return None
    free = (cells >= 0) & (cells <= 20)
    # A false border also protects the map's outer edge.
    distance = ndimage.distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1]*resolution-resolution/2
    safe = free & (distance >= clearance)
    if not safe[row, col]:
        return None
    reachable = np.zeros(cells.shape, bool)
    path_dist = np.full(cells.shape, -1, dtype=np.int32)
    queue = deque([(row, col)])
    reachable[row, col], path_dist[row, col] = True, 0
    while queue:
        y, x = queue.popleft()
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            yy, xx = y+dy, x+dx
            if 0 <= yy < height and 0 <= xx < width and safe[yy, xx] and not reachable[yy, xx]:
                reachable[yy, xx] = True
                path_dist[yy, xx] = path_dist[y, x]+1
                queue.append((yy, xx))
    frontier = free & ndimage.binary_dilation(cells < 0)
    labels, count = ndimage.label(frontier)
    # Find the nearest reachable safe cell for every frontier cell.
    _, indices = ndimage.distance_transform_edt(~reachable, return_indices=True)
    best = None
    for label, slices in enumerate(ndimage.find_objects(labels), 1):
        if slices is None:
            continue
        yy, xx = np.where(labels[slices] == label)
        yy, xx = yy+slices[0].start, xx+slices[1].start
        if len(yy) < min_cluster:
            continue
        sy, sx = indices[:, yy, xx]
        separation = np.hypot(sy-yy, sx-xx)*resolution
        choices = np.argsort(separation)
        for i in choices:
            if separation[i] > clearance+0.3:
                break
            y, x = sy[i], sx[i]
            point = np.array([x+0.5, y+0.5])*resolution @ rot.T + origin_xy
            travel = path_dist[y, x]*resolution
            if travel < 0.5 or np.linalg.norm(point-start_xy) > max_radius or any(np.linalg.norm(point-b) < 0.6 for b in blacklist):
                continue
            score = travel - 0.05*min(len(yy), 100)
            if best is None or score < best[0]:
                direction = np.array([xx[i]-x, yy[i]-y]) @ rot.T
                best = (score, point, math.atan2(direction[1], direction[0]))
            break
    return None if best is None else (best[1], best[2])
