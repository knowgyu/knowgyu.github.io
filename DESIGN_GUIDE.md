# Design Guide

이 문서는 블로그 UI를 어디서 어떻게 수정하는지 빠르게 찾기 위한 메모다.

## 주요 수정 파일

- `_sass/themes/_light.scss`
  라이트 모드 전체 색상 토큰을 정의한다. 배경, 텍스트, 링크, 카드, 사이드바, 탑바 색은 여기서 먼저 조정하면 된다.
- `_sass/themes/_dark.scss`
  다크 모드 색상 토큰 파일이다. 라이트 모드 작업이 끝난 뒤 같은 방식으로 맞추면 된다.
- `_sass/abstracts/_variables.scss`
  기본 폰트 스택, 사이드바 너비, radius 같은 전역 변수 파일이다.
- `_sass/abstracts/_placeholders.scss`
  공통 링크 hover, heading, tag hover 같은 재사용 스타일이 들어 있다.
- `_sass/base/_base.scss`
  `body`, `main`, `.post-preview`, `.content` 같은 전체 레이아웃/기본 타이포그래피를 조정한다.
- `_sass/layout/_sidebar.scss`
  좌측 사이드바 배치, 여백, hover, 프로필 영역 스타일을 바꾼다.
- `_sass/layout/_topbar.scss`
  상단 바의 배경, blur, border, 그림자, 제목 표시를 조정한다.
- `_sass/layout/_panel.scss`
  우측 패널 스타일을 조정한다.
- `_sass/pages/_home.scss`
  홈 화면 전용 스타일 파일이다. 글 카드, 홈 상단 소개, 페이지네이션이 여기 있다.
- `_layouts/home.html`
  홈 화면 실제 마크업이다. 상단 소개 영역이나 글 목록 위에 들어갈 컴포넌트는 여기서 바꾼다.
- `_layouts/default.html`
  메인 레이아웃이다. 사이드바, 탑바, 본문, 패널 배치의 큰 구조가 있다.
- `_config.yml`
  `title`, `tagline`, `theme_mode`, avatar, 소셜 링크 등 사이트 메타 정보는 여기서 바꾼다.

## 현재 라이트 모드 방향

- Pantone 2026 `Cloud Dancer` 를 참고한 거의 흰색에 가까운 오프화이트 베이스
- 전체 배경은 흰색 계열, 보조색은 아주 옅은 하늘색/클라우드 블루
- 과한 hero 대신 얕은 소개 섹션
- 글 카드 중심의 읽기 편한 홈
- 본문은 더 큰 글자와 넉넉한 줄간격으로 읽기 우선
- 폰트는 외부 다운로드 없이 시스템 한글 친화 스택 사용

## 현재 One UI 방향

- 테마 교체 대신 현재 Chirpy 구조 위에 One UI 계열 surface system 을 입히는 방식
- 핵심 원칙:
  - 흰 배경
  - 분명한 섹션 분리
  - 큰 라운드
  - 얕고 부드러운 그림자
  - 푸른 accent 최소 사용
- 주로 손보는 파일:
  - `_sass/themes/_light.scss`
  - `_sass/themes/_dark.scss`
  - `_sass/layout/_sidebar.scss`
  - `_sass/layout/_topbar.scss`
  - `_sass/layout/_panel.scss`
  - `_sass/pages/_home.scss`

## 현재 다크 모드 방향

- 단순 반전이 아니라 남색/슬레이트 기반의 저채도 다크 톤
- 강조색은 라이트 모드와 연결되는 클라우드 블루
- 본문 대비를 충분히 확보해 코드/표/인용문도 읽히는 쪽으로 설계

## 다음에 손보면 좋은 순서

1. `_sass/themes/_light.scss` 에서 색 토큰 먼저 조정
2. `_sass/base/_base.scss`, `_sass/layout/*.scss` 로 전체 프레임 질감 조정
3. `_sass/pages/_home.scss` 또는 `_sass/pages/_post.scss` 에서 페이지별 보정
4. 필요하면 `_layouts/*.html` 에서 마크업 수정

## 참고

- 지금처럼 컨텍스트를 이어가려면 이 파일과 `WORKLOG.md` 를 같이 업데이트하면 충분하다.
- 더 세밀한 추적이 필요하면 날짜별로 `docs/notes/2026-03-13-design.md` 같은 파일을 추가해도 된다.

## Devcontainer 메모

- `.devcontainer/devcontainer.json`
  - `workspaceMount`, `workspaceFolder`: 로컬 레포와 컨테이너 워크스페이스 공유
  - `mounts`: 호스트 `~/.codex` 를 컨테이너 `/home/vscode/.codex` 로 공유
- `.devcontainer/post-create.sh`
  - 컨테이너 생성 후 `@openai/codex` CLI 설치
  - 이후 devcontainer 터미널에서도 `codex` 사용 가능해야 함
