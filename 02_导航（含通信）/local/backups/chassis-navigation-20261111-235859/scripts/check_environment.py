#!/usr/bin/env python3
"""Read-only local dependency, resource and connection inventory; no hardware I/O."""
import argparse
import importlib
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_prefix


def command(*args):
    result = subprocess.run(args, capture_output=True, text=True, timeout=15)
    return result.returncode, (result.stdout + result.stderr).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    root = Path(os.environ['RM_NAV_ROOT'])
    runtime = Path(os.environ['RM_NAV_RUNTIME'])
    report = {'software_errors': [], 'packages': {}, 'python': {}, 'executables': {},
              'pending_hardware': [], 'configuration': {}}
    errors = report['software_errors']
    for manifest in sorted((root/'src').rglob('package.xml')):
        for dependency in ET.parse(manifest).getroot():
            if dependency.tag not in ('depend', 'exec_depend'):
                continue
            name = dependency.text.strip()
            if name in report['packages']:
                continue
            if name.startswith(('lib', 'python3-')):
                code, value = command('dpkg-query', '-W', '-f=${Status}', name)
                installed = code == 0 and value == 'install ok installed'
                report['packages'][name] = value
            else:
                try:
                    value = get_package_prefix(name)
                    installed = True
                except LookupError:
                    installed, value = False, 'missing ROS package'
                report['packages'][name] = value
            if not installed:
                errors.append('Missing runtime dependency: '+name)
    for name in ('rclpy', 'yaml', 'numpy', 'sensor_msgs_py.point_cloud2',
                 'livox_ros_driver2.msg', 'rosbag2_py'):
        try:
            importlib.import_module(name)
            report['python'][name] = 'OK'
        except ImportError as error:
            report['python'][name] = str(error)
            errors.append('Python dependency: '+name)
    for package, executable in (
        ('rm_chassis_comm', 'chassis_node'), ('rm_chassis_comm', 'sentry_frame_bridge'),
        ('livox_ros_driver2', 'livox_ros_driver2_node'), ('point_lio', 'pointlio_mapping'),
        ('small_gicp_relocalization', 'small_gicp_relocalization_node'),
        ('slam_toolbox', 'sync_slam_toolbox_node'), ('nav2_map_server', 'map_server'),
        ('rviz2', 'rviz2')):
        try:
            path = Path(get_package_prefix(package))/'lib'/package/executable
            if not path.is_file():
                raise RuntimeError('Missing executable: '+str(path))
            code, libraries = command('ldd', str(path))
            missing = [line.strip() for line in libraries.splitlines() if 'not found' in line]
            if code != 0 or missing:
                raise RuntimeError('Unresolved dynamic libraries: '+str(missing or libraries))
            report['executables'][package+'/'+executable] = 'OK'
        except (LookupError, RuntimeError) as error:
            report['executables'][package+'/'+executable] = str(error)
            errors.append(str(error))
    for package in ('rm_chassis_comm', 'fake_vel_transform', 'pb2025_nav_bringup'):
        prefix = report['packages'].get(package)
        if prefix is None:
            try:
                prefix = get_package_prefix(package)
            except LookupError:
                prefix = ''
        if not str(prefix).startswith(str(runtime/'install')+'/'):
            errors.append('Wrong package overlay: '+package+'='+str(prefix))
    frames = yaml.safe_load((root/'local/sentry_real.yaml').read_text())
    nav = yaml.safe_load((root/'local/nav2_params.yaml').read_text())
    radar_path = Path(nav['livox_ros_driver2']['ros__parameters']['user_config_path'])
    if not radar_path.is_file():
        errors.append('Missing radar configuration: '+str(radar_path))
    else:
        radar = json.loads(radar_path.read_text())
        report['configuration']['lidar_ips'] = [item['ip'] for item in radar['lidar_configs']]
        if 'Mid360s' not in radar:
            errors.append('Radar configuration does not contain Mid360s')
    report['configuration']['geometry_confirmed'] = frames['frames']['geometry_confirmed']
    if not frames['frames']['geometry_confirmed']:
        report['pending_hardware'].append('Gimbal pivots/zero/sign calibration unconfirmed; motion inhibited')
    for name in ('map.yaml', 'map.pgm', 'map.pcd'):
        if not (root/'local/maps/20261110_210446'/name).is_file():
            errors.append('Missing example map: '+name)
    code, value = command('ip', '-br', 'addr', 'show', 'enp89s0')
    report['network'] = value
    carrier = Path('/sys/class/net/enp89s0/carrier')
    try:
        link_up = carrier.read_text().strip() == '1'
    except OSError:
        link_up = False
    if not link_up:
        report['pending_hardware'].append('Lidar Ethernet link is down')
    devices = []
    for path in sorted(Path('/dev/serial/by-id').glob('*')):
        _, props = command('udevadm', 'info', '--query=property', '--name='+str(path))
        debugger = any(word in props.lower() for word in ('cmsis', 'daplink', 'muselab', 'stlink', 'st-link'))
        devices.append({'path': str(path), 'target': str(path.resolve()), 'debugger': debugger})
    report['usb_devices'] = devices
    if not any(not device['debugger'] for device in devices):
        report['pending_hardware'].append('No application USB serial candidate; verify C-board firmware/USB')
    report['software_ready'] = not errors
    report['note'] = 'Read-only inventory. USB candidates are not proof of the new protocol; no physical port opened.'
    output = Path(args.output) if args.output else root/'local/environment-check.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if not errors else 1


if __name__ == '__main__':
    raise SystemExit(main())
