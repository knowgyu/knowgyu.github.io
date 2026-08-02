# knowgyu.github.io design contract

## Product
한국어 기술 현장 노트: 로보틱스, 컴퓨터 비전, MLOps, AI 작업 기록을 사람이 읽고 다시 찾기 쉽게 정리한다.

## Principles
- Content before chrome; article titles, prose, code, tables, and metadata establish hierarchy.
- Use one Notion-style blue accent for links and focus; selection and hover stay warm neutral.
- Prefer flat editorial lists and whitespace over cards, gradients, blur, hover lift, or decorative elevation.
- Preserve Jekyll/Chirpy routes, search, archives, categories, pagination, TOC, and light/dark behavior.

## Ownership
- `_data/locales` and `_tabs` own labels and route order.
- `_includes/sidebar.html` owns desktop rail and mobile drawer navigation.
- `_includes/topbar.html` owns breadcrumb, search, and mobile trigger only.
- `_sass/themes` owns semantic color values; layout and page styles consume those tokens.
- Shadows are reserved for overlays. Syntax/status colors are explicit exceptions.

## Color grammar
- Light mode uses a white reading canvas, warm-white (`#f6f5f4`) navigation chrome, near-black text, and low-alpha warm-gray dividers.
- Dark mode uses a `#191919` reading canvas, a visibly separate `#202020` rail, soft-white text, and neutral gray interaction states.
- `--main-bg`, `--sidebar-bg`, `--surface-color`, `--surface-muted-color`, `--border-color`, and `--accent-color` are the canonical visual roles. Page styles consume them instead of inventing route-specific palettes.
- Blue is the only saturated chrome color. It is reserved for links, focus, disclosure icons, and the active-rail indicator; ordinary hover and selected fills are neutral.

## Responsive and accessibility
The desktop rail is approximately 208px and neutral. Mobile uses the existing drawer and trigger. Reading remains single-column, zoomable, keyboard-focusable, and free of page-wide overflow at 360px.

## Forbidden patterns
No unavailable chatbot by default, a disabled viewport-scaling directive, decorative layout gradients, glass blur, large surface shadows, or new legacy console-prefixed tokens. Do not add dependencies or rewrite post content/front matter.
