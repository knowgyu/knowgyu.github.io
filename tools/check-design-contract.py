#!/usr/bin/env python3
"""Small drift guard for the human-first editorial contract."""
from pathlib import Path
import re, sys
root = Path(__file__).resolve().parents[1]
errors=[]
if not (root/'DESIGN.md').exists(): errors.append('DESIGN.md is missing')
# Inspect authored web sources only; generated sites, vendor bundles, and research
# notes may legitimately quote the patterns this guard is meant to prevent.
source_roots = [root / name for name in ('_includes', '_layouts', '_sass', 'assets')]
for p in (p for directory in source_roots for p in directory.rglob('*')):
    if not p.is_file() or p.name == 'check-design-contract.py': continue
    try: s=p.read_text()
    except UnicodeDecodeError: continue
    definitions = []
    depth = 0
    for line in s.splitlines():
        if depth <= 1:
            definitions.extend(re.findall(r'--([a-z][\w-]*)\s*:', line))
        depth += line.count('{') - line.count('}')
    for token in sorted(set(definitions)):
        if definitions.count(token) > 1:
            errors.append(f'{p}: duplicate --{token} definition')
    for token in definitions:
        if token.startswith('card-'):
            errors.append(f'{p}: legacy --{token} token')
    if 'user-scalable=no' in s: errors.append(f'{p}: user-scalable=no')
    if re.search(r'--console-[\w-]+', s): errors.append(f'{p}: obsolete --console-* token')
    if '--card-hovor-bg' in s: errors.append(f'{p}: misspelled --card-hovor-bg token')
for p in (root/'_sass/layout', root/'_sass/pages'):
    for f in p.glob('*.scss'):
        s=f.read_text()
        if 'gradient(' in s: errors.append(f'{f}: layout/page gradient')

# Keep decorative effects out of the page shell; image/syntax/status effects remain local.
base = (root / '_sass/base/_base.scss').read_text()
body = re.search(r'body\s*\{(?P<body>.*?)\n\}', base, re.S)
if body and 'gradient(' in body.group('body'):
    errors.append(f'{root / "_sass/base/_base.scss"}: body gradient')
if body and re.search(r'overflow-x\s*:\s*hidden', body.group('body')):
    errors.append(f'{root / "_sass/base/_base.scss"}: body overflow mask')

# Lock the canvas/rail contrast and singular accent that define the Notion-like palette.
for label, path, fragment in (
    ('light Notion canvas', root / '_sass/themes/_light.scss', '--main-bg: #ffffff;'),
    ('light warm rail', root / '_sass/themes/_light.scss', '--surface-muted-color: #f6f5f4;'),
    ('light singular accent', root / '_sass/themes/_light.scss', '--accent-color: #0075de;'),
    ('dark Notion canvas', root / '_sass/themes/_dark.scss', '--main-bg: #191919;'),
    ('dark distinct rail', root / '_sass/themes/_dark.scss', '--surface-muted-color: #202020;'),
    ('dark singular accent', root / '_sass/themes/_dark.scss', '--accent-color: #529cca;'),
):
    if fragment not in path.read_text():
        errors.append(f'{label}: missing {fragment!r} in {path}')

for f in (root / '_sass/themes').glob('*.scss'):
    s = f.read_text()
    for token in ('--sidebar-bg', '--intro-bg'):
        match = re.search(rf'{re.escape(token)}\s*:\s*([^;]+)', s)
        if match and 'gradient(' in match.group(1):
            errors.append(f'{f}: {token} gradient')
    for token in ('--soft-shadow', '--card-shadow', '--card-shadow-hover'):
        match = re.search(rf'{re.escape(token)}\s*:\s*([^;]+)', s)
        if match and match.group(1).strip() != 'none':
            errors.append(f'{f}: {token} must be none')
for directory in (root / '_sass/base', root / '_sass/layout', root / '_sass/pages'):
    for f in directory.glob('*.scss'):
        if f.name == '_syntax.scss':
            continue
        s = f.read_text()
        if re.search(r'box-shadow\s*:\s*var\(--card-shadow(?:-hover)?\b', s):
            errors.append(f'{f}: non-overlay card shadow')
if errors:
    print('\n'.join(errors)); sys.exit(1)
print('design contract: PASS')
