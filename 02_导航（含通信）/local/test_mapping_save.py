"""Synthetic map file validation; isolated ROS domain, no real sensor/control."""
import importlib.util
import json
import math
from pathlib import Path
import tempfile

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
import yaml


root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('map_recorder', root/'scripts/mid360_map_recorder.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
(root/'local/test-artifacts').mkdir(exist_ok=True)
output = Path(tempfile.mkdtemp(prefix='synthetic-map-', dir=root/'local/test-artifacts'))
rclpy.init(args=['--ros-args', '-p', 'output_dir:='+str(output)])
node = module.MapRecorder()
checks = []
try:
    response = node.on_save(Trigger.Request(), Trigger.Response())
    assert not response.success and not (output/'map.pcd').exists()
    checks.append('Empty input rejected without creating a map')
    points = np.array([[.01,.02,.30], [.04,.06,.34], [1.,0.,.5], [np.nan,0.,0.]], dtype=np.float32)
    cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='wrong_frame'), points)
    node.on_cloud(cloud)
    assert not node.voxels
    checks.append('Cloud in incorrect frame rejected')
    cloud.header.frame_id = 'camera_init'
    node.on_cloud(cloud)
    assert len(node.voxels) == 2 and node.frames == 1
    response = node.on_save(Trigger.Request(), Trigger.Response())
    assert not response.success and (output/'map.pcd').is_file()
    assert not (output/'map.yaml').exists()
    checks.append('NaNs removed and points voxelized; 3D-only save reports incomplete 2D map')
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width, grid.info.height, grid.info.resolution = 3, 2, .05
    grid.info.origin.position.x, grid.info.origin.position.y = 1., -2.
    grid.info.origin.orientation.z = math.sin(math.pi/4)
    grid.info.origin.orientation.w = math.cos(math.pi/4)
    grid.data = [-1, 0, 100, 25, 65, 50]
    node.on_grid(grid)
    response = node.on_save(Trigger.Request(), Trigger.Response())
    assert response.success
    pcd_header, pcd_data = (output/'map.pcd').read_bytes().split(b'DATA binary\n', 1)
    assert b'POINTS 2\n' in pcd_header and len(pcd_data) == 24
    np.testing.assert_allclose(np.frombuffer(pcd_data, dtype='<f4').reshape(-1,3), node.cloud_array())
    assert (output/'map.pgm').read_bytes() == b'P5\n3 2\n255\n'+bytes([254,0,205,205,254,0])
    metadata = yaml.safe_load((output/'map.yaml').read_text())
    np.testing.assert_allclose(metadata['origin'], [1.,-2.,math.pi/2])
    assert math.isclose(metadata['resolution'], .05, rel_tol=1e-6)
    summary = json.loads((output/'summary.json').read_text())
    assert summary['grid']['known_cells'] == 5 and summary['grid']['occupied_cells'] == 2
    assert not list(output.glob('*.tmp'))
    checks.append('PCD binary data, PGM row order/cell values, YAML origin/yaw/resolution verified')
    report = {'passed': True, 'synthetic_data_only': True, 'output': str(output), 'checks': checks}
    (root/'local/mapping-save-test.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
finally:
    node.destroy_node()
    rclpy.shutdown()
