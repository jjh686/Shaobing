"""Defaults for sentry_bringup. No ROS imports; safe to unit-test on the host."""

import os
from datetime import datetime
from pathlib import Path, PurePosixPath


def append_path(base, *parts):
    base = str(base)
    if base.startswith('/') and not (len(base) >= 2 and base[1] == ':'):
        path = PurePosixPath(base)
        for part in parts:
            path /= part
        return str(path)
    path = Path(base)
    for part in parts:
        path /= part
    return str(path)


VISION_DIRNAME = '01_自瞄（含通信）'
NAV_DIRNAME = '02_导航（含通信）'
DECISION_DIRNAME = '06_决策'
VISION_SCRIPT = Path('scripts') / 'run_live_usb_sp.sh'
LIVE_PATH_KEYS = ('source_root', 'runtime', 'mounting_config', 'output_dir')


def is_repo_root(root):
    root = Path(root)
    return all((root / name).is_dir() for name in (VISION_DIRNAME, NAV_DIRNAME, DECISION_DIRNAME))


def repo_root(env=None, fallback=None):
    env = os.environ if env is None else env
    candidates = []
    if env.get('RM_11_13_ROOT'):
        candidates.append(Path(env['RM_11_13_ROOT']))
    if env.get('RM_NAV_ROOT'):
        candidates.append(Path(env['RM_NAV_ROOT']).resolve().parent)
    if env.get('RM_DECISION_ROOT'):
        candidates.append(Path(env['RM_DECISION_ROOT']).resolve().parent)
    if fallback:
        candidates.append(Path(fallback))
    for item in candidates:
        try:
            resolved = item.resolve()
        except OSError:
            continue
        if is_repo_root(resolved):
            return resolved
    return None


def fill_live_paths(values, env=None, now=None):
    """Fill empty live path arguments from the nav environment.

    This does not read or rewrite live_mounting.yaml, and does not mark
    an unconfirmed mounting as ready.
    """
    env = os.environ if env is None else env
    now = now or datetime.now()
    runtime = str(values.get('runtime') or env.get('RM_NAV_RUNTIME') or '')
    source = str(values.get('source_root') or '')
    if not source and runtime:
        source = append_path(runtime, 'source')
    if not source and env.get('RM_NAV_ROOT'):
        source = env['RM_NAV_ROOT']
        if not runtime:
            runtime = env.get('RM_NAV_RUNTIME') or append_path(Path.home(), '.cache', 'rm_sentry_nav')
    mounting = str(values.get('mounting_config') or '')
    if not mounting and source:
        mounting = append_path(source, 'local', 'live_mounting.yaml')
    output = str(values.get('output_dir') or '')
    if not output:
        nav_root = env.get('RM_NAV_ROOT') or (str(Path(source).resolve()) if source else '')
        if nav_root:
            output = append_path(nav_root, 'local', 'maps', now.strftime('%Y%m%d_%H%M%S'))
    return {
        'source_root': source,
        'runtime': runtime,
        'mounting_config': mounting,
        'output_dir': output,
    }


def resolve_vision_project(explicit='', env=None, fallback_repo=None):
    if explicit:
        path = Path(explicit)
        if not path.is_dir():
            raise FileNotFoundError('vision_project is not a directory: ' + str(path))
        return path
    root = repo_root(env=env, fallback=fallback_repo)
    if root is None:
        raise FileNotFoundError(
            'Cannot resolve 01_自瞄（含通信）. Set vision_project or RM_11_13_ROOT, '
            'or run scripts/run_sentry_bringup.sh from the source tree.')
    project = root / VISION_DIRNAME
    if not project.is_dir():
        raise FileNotFoundError('Missing vision project: ' + str(project))
    return project


def vision_command(project, usb=True):
    script = Path(project) / VISION_SCRIPT
    if not script.is_file():
        raise FileNotFoundError('Missing official vision entry: ' + str(script))
    cmd = ['bash', str(script)]
    if not usb:
        cmd.append('--preview')
    return cmd


def mounting_refused_message(exc):
    return (
        '实车实时导航未启动：安装参数尚未确认。'
        '需要 live_mounting.yaml 中 configured、fixed_gimbal、footprint_confirmed 均为 true，'
        '且 translation_m 与 rotation_rpy_rad 为有限数值。'
        '本启动不会改写 live_mounting.yaml，也不会把未确认的安装当作已标定。'
        ' 原始错误：' + str(exc)
    )
