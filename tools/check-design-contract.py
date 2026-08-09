#!/usr/bin/env python3
"""Small drift guard for the Tistory-like editorial shell contract."""
from pathlib import Path
import re, sys
root = Path(__file__).resolve().parents[1]
errors=[]

def hex_color(text, token):
    match = re.search(rf'{re.escape(token)}\s*:\s*(#[0-9a-fA-F]{{6}})', text)
    return match.group(1) if match else ''

def contrast(a, b):
    def lum(hex_value):
        parts = [int(hex_value[i:i + 2], 16) / 255 for i in (1, 3, 5)]
        channels = [v / 12.92 if v <= 0.03928 else ((v + 0.055) / 1.055) ** 2.4 for v in parts]
        return 0.2126 * channels[0] + 0.7152 * channels[1] + 0.0722 * channels[2]
    x, y = sorted((lum(a), lum(b)), reverse=True)
    return (x + 0.05) / (y + 0.05)
forbidden_site_paths = [
    root / '_site' / 'AGENTS.md',
    root / '_site' / 'DESIGN.md',
    root / '_site' / 'embedding_scripts.sh',
    root / '_site' / '마크다운파일변환기.py',
    root / '_site' / 'tests',
]
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
    if 'sidebar-collapse-toggle' in s or 'knowgyu:sidebar-collapsed' in s or 'sidebar-compact' in s:
        errors.append(f'{p}: sidebar compact/collapse state must be removed')
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

# Lock the canvas/rail contrast and singular accent that define the authored shell.
for label, path, fragment in (
    ('light warm rail', root / '_sass/themes/_light.scss', '--surface-muted-color: #f6f5f4;'),
    ('light singular accent', root / '_sass/themes/_light.scss', '--accent-color: #0075de;'),
    ('dark reading canvas', root / '_sass/themes/_dark.scss', '--main-bg: #191919;'),
    ('dark distinct rail', root / '_sass/themes/_dark.scss', '--surface-muted-color: #202020;'),
    ('dark singular accent', root / '_sass/themes/_dark.scss', '--accent-color: #529cca;'),
):
    if fragment not in path.read_text():
        errors.append(f'{label}: missing {fragment!r} in {path}')

light_theme = (root / '_sass/themes/_light.scss').read_text()
light_main = hex_color(light_theme, '--main-bg')
light_text = hex_color(light_theme, '--text-color')
if not light_main:
    errors.append(f'{root / "_sass/themes/_light.scss"}: --main-bg hex token missing')
elif light_main.lower() not in {'#ffffff', '#fbfaf8', '#faf9f6', '#f8f7f4', '#f7f5ef'}:
    errors.append(f'{root / "_sass/themes/_light.scss"}: --main-bg must stay white or approved warm off-white')
if light_main and light_text and contrast(light_main, light_text) < 4.5:
    errors.append(f'{root / "_sass/themes/_light.scss"}: --main-bg/--text-color contrast below 4.5:1')

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
for path in forbidden_site_paths:
    if path.exists():
        errors.append(f'{path}: forbidden generated output')
if errors:
    print('\n'.join(errors)); sys.exit(1)
print('design contract: PASS')
