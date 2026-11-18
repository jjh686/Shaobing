#!/usr/bin/env python3
"""Confirm that input directories have not changed since independent copies."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root/'docs/source_copies_manifest.json').read_text())
report = {'passed': True, 'originals': []}
for item in manifest['sources']:
    original = Path(item['original'])
    expected = {entry['path']: entry for entry in item['files']}
    present = {str(p.relative_to(original)): p for p in original.rglob('*')
               if p.is_file() or p.is_symlink()}
    differences = []
    for name, entry in expected.items():
        p = present.get(name)
        if p is None:
            differences.append({'path': name, 'change': 'missing'})
        elif 'symlink' in entry:
            if not p.is_symlink() or str(p.readlink()) != entry['symlink']:
                differences.append({'path': name, 'change': 'symlink changed'})
        elif p.is_symlink() or hashlib.sha256(p.read_bytes()).hexdigest() != entry['sha256']:
            differences.append({'path': name, 'change': 'contents changed'})
    differences.extend({'path': name, 'change': 'added'} for name in sorted(present.keys()-expected.keys()))
    report['originals'].append({'path': str(original), 'checked_files': len(expected),
                                'differences': differences, 'unchanged': not differences})
    report['passed'] &= not differences
(root/'logs/source-integrity.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+chr(10))
print(json.dumps(report, ensure_ascii=False, indent=2))
raise SystemExit(0 if report['passed'] else 1)
