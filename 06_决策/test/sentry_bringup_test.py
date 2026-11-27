import sys
import unittest
from datetime import datetime
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'launch'))

from bringup_defaults import (
    fill_live_paths, is_repo_root, mounting_refused_message, repo_root,
    resolve_vision_project, vision_command,
)


REPO = Path(__file__).resolve().parents[2]


class BringupDefaultsTest(unittest.TestCase):
    def test_repo_layout_is_detected_from_env(self):
        self.assertTrue(is_repo_root(REPO))
        self.assertEqual(repo_root(env={'RM_11_13_ROOT': str(REPO)}), REPO.resolve())
        self.assertEqual(repo_root(env={'RM_NAV_ROOT': str(REPO / '02_导航（含通信）')}), REPO.resolve())
        self.assertEqual(
            repo_root(env={'RM_DECISION_ROOT': str(REPO / '06_决策')}), REPO.resolve())
        self.assertIsNone(repo_root(env={'RM_11_13_ROOT': str(REPO / 'missing')}, fallback=None))

    def test_live_paths_come_from_nav_environment_without_inventing_mounting(self):
        now = datetime(2026, 11, 27, 14, 20, 0)
        env = {'RM_NAV_ROOT': '/opt/nav', 'RM_NAV_RUNTIME': '/tmp/runtime'}
        filled = fill_live_paths({}, env=env, now=now)
        self.assertEqual(filled['runtime'], '/tmp/runtime')
        self.assertEqual(filled['source_root'], '/tmp/runtime/source')
        self.assertEqual(filled['mounting_config'], '/tmp/runtime/source/local/live_mounting.yaml')
        self.assertEqual(filled['output_dir'], '/opt/nav/local/maps/20261127_142000')
        explicit = fill_live_paths({
            'source_root': '/explicit/src',
            'runtime': '/explicit/run',
            'mounting_config': '/explicit/live_mounting.yaml',
            'output_dir': '/explicit/out',
        }, env=env, now=now)
        self.assertEqual(explicit['source_root'], '/explicit/src')
        self.assertEqual(explicit['mounting_config'], '/explicit/live_mounting.yaml')
        self.assertEqual(explicit['output_dir'], '/explicit/out')

    def test_vision_command_uses_official_script_and_keeps_usb_for_status(self):
        project = resolve_vision_project(env={'RM_11_13_ROOT': str(REPO)})
        self.assertEqual(project, REPO / '01_自瞄（含通信）')
        usb = vision_command(project, usb=True)
        preview = vision_command(project, usb=False)
        self.assertEqual(usb[0], 'bash')
        self.assertTrue(usb[1].endswith('run_live_usb_sp.sh'))
        self.assertEqual(usb, [usb[0], usb[1]])
        self.assertEqual(preview, [usb[0], usb[1], '--preview'])

    def test_unconfirmed_mounting_message_does_not_suggest_editing_yaml(self):
        text = mounting_refused_message(ValueError('Confirm measured mounting'))
        self.assertIn('安装参数尚未确认', text)
        self.assertIn('不会改写 live_mounting.yaml', text)
        self.assertIn('Confirm measured mounting', text)


if __name__ == '__main__':
    unittest.main()
