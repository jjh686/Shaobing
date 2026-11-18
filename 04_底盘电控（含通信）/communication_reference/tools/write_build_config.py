#!/usr/bin/env python3
"""Record source configuration defaults and the binaries from this build."""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parents[1]
settings={
    'chassis_firmware/modules/nav_relay/nav_relay_config.h':('NAV_RELAY_ALLOW_MOTION','NAV_AXES_VERIFIED'),
    'gimbal_firmware/application/gimbal/gimbal_config.h':('GIMBAL_INSTALLATION_CONFIRMED',),
    'gimbal_firmware/application/communication/nav_joint_config.h':('NAV_JOINT_CALIBRATED',),
}
defaults={}
for header,names in settings.items():
    output=subprocess.check_output(['arm-none-eabi-gcc','-dM','-E','-x','c','-include',str(root/header),'/dev/null'],text=True)
    macros={parts[1]:parts[2] for line in output.splitlines() if len(parts:=line.split(maxsplit=2))==3 and parts[0]=='#define'}
    defaults.update({name:int(macros[name]) for name in names})
report={
    'built_at':datetime.datetime.now().astimezone().isoformat(),
    'compiler':subprocess.check_output(['arm-none-eabi-gcc','--version'],text=True).splitlines()[0],
    'defaults':defaults,
    'gimbal':'STM32F407 / navigation CAN1 1Mbps to chassis CAN2 / gimbal motors CAN2 (locked) / USB 0483:00ca RM Nav Convert SP',
    'chassis':'STM32F407 / CAN2 1Mbps / CAN1 M3508 IDs LF1 RF2 LB4 RB3',
    'binaries':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (root/'artifacts').glob('*.bin')},
    'note':'Build output only; this script does not flash hardware or validate physical navigation.',
}
(root/'artifacts/build_config.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
