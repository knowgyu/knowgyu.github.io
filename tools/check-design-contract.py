#!/usr/bin/env python3
"""Small drift guard for the human-first editorial contract."""
from pathlib import Path
import re, sys
root = Path(__file__).resolve().parents[1]
errors=[]
if not (root/'DESIGN.md').exists(): errors.append('DESIGN.md is missing')
for p in root.rglob('*'):
    if not p.is_file() or '.git' in p.parts or p.name == 'check-design-contract.py': continue
    try: s=p.read_text()
    except UnicodeDecodeError: continue
    if 'user-scalable=no' in s: errors.append(f'{p}: user-scalable=no')
    if re.search(r'--console-[\w-]+', s): errors.append(f'{p}: obsolete --console-* token')
for p in (root/'_sass/layout', root/'_sass/pages'):
    for f in p.glob('*.scss'):
        s=f.read_text()
        if 'gradient(' in s: errors.append(f'{f}: layout/page gradient')
if errors:
    print('\n'.join(errors)); sys.exit(1)
print('design contract: PASS')
