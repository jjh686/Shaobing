"""Check the selected fixed-chassis launch in composition mode; no hardware."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

assert os.environ.get('ROS_DOMAIN_ID') == '95'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
root = Path(__file__).resolve().parents[1]
report = {'hardware_opened': False, 'use_composition': True, 'passed': False}
with (root/'local/chassis-composed-startup-20261112.log').open('w') as log:
    process = subprocess.Popen(['bash', str(root/'scripts/run_chassis_nav.sh'),
        'check', 'use_composition:=True'], stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    try:
        time.sleep(5)
        assert process.poll() is None, 'launch exited before observation'
        observation = root/'local/chassis-composed-observation-20261112.json'
        result = subprocess.run(['bash', str(root/'scripts/observe_navigation.sh'),
            '--profile', 'chassis', '--duration', '3', '--output', str(observation)],
            capture_output=True, text=True, timeout=15)
        assert result.returncode == 0, result.stderr
        data = json.loads(observation.read_text())
        names = data['nodes']
        for name in ('chassis_comm','controller_server','planner_server','behavior_server','bt_navigator','velocity_smoother'):
            assert '/red_standard_robot1/'+name in names, name
        assert not any('fake_vel_transform' in n or 'sentry_frame_bridge' in n for n in names)
        assert all(v == 'unconfigured' for v in data['lifecycle'].values()), data['lifecycle']
        assert data['tf']['base_footprint <- front_mid360']['xyz'] == [0.,0.,.2]
        values=data['diagnostics'][0]['values']
        for key, value in {'enabled':'false','route':'2','frame_id':'base_footprint',
            'require_gimbal_state':'false','require_frame_ready':'false','use_stamped_commands':'false'}.items():
            assert values[key] == value, (key,values.get(key))
        report.update(passed=True, nodes=len(names), lifecycle=data['lifecycle'],
                      diagnostics=values)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            report['launcher_exit']=process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM)
            report['launcher_exit']=process.wait(timeout=5)
        (root/'local/chassis-composed-startup-result-20261112.json').write_text(
            json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
