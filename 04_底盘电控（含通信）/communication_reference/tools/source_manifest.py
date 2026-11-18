#!/usr/bin/env python3
"""Record exact source hashes and diffs against the user's two input trees."""
import difflib
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
inputs = json.loads((root/'docs/input_paths.json').read_text())
excluded = {'.git','build','build2','build_debug','cmake-build-debug','cmake-build-release',
            '__pycache__','.cache','原始改动文件','.github','.vscode','.gitignore'}


def files(base):
    result = {}
    for path in base.rglob('*'):
        relative = path.relative_to(base)
        if any(p in excluded or p.startswith(('build-','build_')) or p.endswith('_build') for p in relative.parts):
            continue
        if path.is_file() and path.suffix not in ('.o','.obj','.d','.pyc','.elf','.bin','.hex','.map'):
            result[relative.as_posix()] = path
    return result


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


report = {'input_paths':inputs,'comparisons':{},'additional_sources':{},
          'excluded':'Build output, binaries, caches, original-change backups, Git/CI/editor metadata'}
(root/'docs/patches').mkdir(exist_ok=True)
for label, original, delivered in (
    ('gimbal', Path(inputs['gimbal']), root/'gimbal_firmware'),
    ('chassis', Path(inputs['chassis_original']), root/'chassis_firmware'),
    ('pc', Path(inputs['navigation']), root/'pc/source/rm_chassis_comm'),
):
    before, after = files(original), files(delivered)
    entries=[];patch=[]
    for name in sorted(before.keys()|after.keys()):
        old,new = before.get(name),after.get(name)
        old_hash,new_hash = sha(old) if old else None,sha(new) if new else None
        state='unchanged' if old_hash==new_hash else 'added' if not old else 'removed' if not new else 'modified'
        entries.append(dict(path=name,state=state,original_sha256=old_hash,delivered_sha256=new_hash))
        if state=='unchanged':continue
        try:
            a=old.read_text().splitlines(True) if old else []
            b=new.read_text().splitlines(True) if new else []
        except UnicodeError:
            continue
        patch.extend(difflib.unified_diff(a,b,fromfile='a/'+name if old else '/dev/null',
                                        tofile='b/'+name if new else '/dev/null'))
    report['comparisons'][label]=entries
    (root/'docs/patches'/f'{label}.patch').write_text(''.join(patch))
for folder in ('common','pc','tests','tools'):
    for name,path in files(root/folder).items():
        report['additional_sources'][folder+'/'+name]=sha(path)
(root/'docs/source_changes.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
for label,entries in report['comparisons'].items():
    counts={s:sum(e['state']==s for e in entries) for s in ('modified','added','removed','unchanged')}
    print(label,counts)
