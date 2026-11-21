from glob import glob
from setuptools import setup

setup(name='rm_live_nav', version='0.1.0', packages=['rm_live_nav'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/rm_live_nav']),
                  ('share/rm_live_nav', ['package.xml']),
                  ('share/rm_live_nav/launch', glob('launch/*.launch.py')),
                  ('share/rm_live_nav/config', glob('config/*'))],
      entry_points={'console_scripts': [
          'lio_frames = rm_live_nav.lio_frames:main',
          'motion_gate = rm_live_nav.motion_gate:main',
          'frontier_explorer = rm_live_nav.frontier_explorer:main']})
