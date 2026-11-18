#!/usr/bin/env python3
"""Keep identical wire codecs on both MCUs; common/ is the source of truth."""
from pathlib import Path
import shutil
root = Path(__file__).resolve().parents[1]
for board in ('gimbal_firmware', 'chassis_firmware'):
    target = root / board / 'modules/nav_conversion'
    target.mkdir(parents=True, exist_ok=True)
    for source in (root/'common').rglob('*'):
        if source.is_file():
            dest = target / source.relative_to(root/'common')
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, dest)
