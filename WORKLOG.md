# Worklog

## 2026-03-13

### Devcontainer

- `.devcontainer/devcontainer.json` 에 `workspaceMount`, `workspaceFolder` 를 명시적으로 추가했다.
- 목적: 로컬 레포와 컨테이너 워크스페이스가 같은 파일을 바라보는 설정을 분명히 하기 위함.
- `.devcontainer/devcontainer.json` 에 `remoteUser: "vscode"` 와 `~/.codex` bind mount 를 추가했다.
- 목적: devcontainer 안에서도 호스트와 같은 Codex 인증/설정을 재사용하기 위함.
- `.devcontainer/post-create.sh` 에 Codex CLI 설치를 추가했다.

### Blog light mode redesign

- 기존 홈 hero / featured 영역은 시각적으로 부담이 커서 제거했다.
- 홈 상단은 `_layouts/home.html` 의 `home-intro` 소개 블록으로 축소했다.
- 라이트 모드 전체 팔레트를 `_sass/themes/_light.scss` 에서 다시 재정의했다.
  - 크림 톤은 폐기
  - Pantone 2026 `Cloud Dancer` 참고의 오프화이트 베이스로 수정
  - 아주 옅은 하늘색 계열을 보조색으로 사용
- 폰트 스택을 `_sass/abstracts/_variables.scss` 와 `_sass/base/_base.scss` 에서 시스템 한글 친화 스택으로 정리했다.
- 링크 hover 공통 색은 `_sass/abstracts/_placeholders.scss` 에서 테마 변수 기반으로 바꿨다.
- 사이드바 / 탑바 질감은 `_sass/layout/_sidebar.scss`, `_sass/layout/_topbar.scss` 에서 과한 유리 효과를 줄이고 톤을 맞췄다.
- 홈 글 카드와 페이지네이션은 `_sass/pages/_home.scss` 에서 더 차분하게 재조정했다.
- 본문 가독성을 위해 `_sass/base/_base.scss`, `_sass/base/_typography.scss`, `_sass/base/_syntax.scss`, `_sass/pages/_post.scss`, `_layouts/post.html` 도 수정했다.
  - 기본 폰트 크기 상향
  - 본문 줄간격 확대
  - 포스트 본문 폭 제한
  - 코드블록 / 인용문 / 제목 크기 재조정

### Blog dark mode redesign

- `_sass/themes/_dark.scss` 를 라이트 모드와 같은 시스템으로 다시 설계했다.
- 남색/슬레이트 기반 배경과 클라우드 블루 accent 를 사용해 다크 모드도 별도 완성도 있게 보이도록 조정했다.

### Verification status

- `bundle exec jekyll build` 확인은 현재 실행 환경에 `bundle` 명령이 없어서 수행하지 못했다.
- 실제 렌더 검증은 devcontainer 안에서 다시 열어 확인해야 한다.

### Next candidates

- 다크 모드 `_sass/themes/_dark.scss` 톤 재설계
- 포스트 상세 페이지 `_sass/pages/_post.scss` 색/타이포 정리
- 우측 패널 `_sass/layout/_panel.scss` 톤 통일

## 2026-03-14

### Typography tuning

- 제목 스케일이 과해 보이던 부분을 다시 줄였다.
- `_sass/base/_base.scss`
  - 기본 루트 폰트 크기 하향
  - 본문 글자 크기와 줄간격 소폭 축소
- `_sass/base/_typography.scss`
  - `h2` 이하 헤딩 간격과 크기 계수 축소
  - blockquote 여백 축소
- `_sass/pages/_post.scss`
  - 포스트 제목, 설명, 메타 텍스트 크기 축소
  - TOC 텍스트 크기 축소
- `_sass/pages/_home.scss`
  - 홈 소개 타이틀과 카드 제목 크기 축소

### Color tuning

- muted 계열 색이 너무 많은 문제를 줄이기 위해 라이트 모드의 보조 텍스트 색을 조금 진하게 조정했다.
- 대상 파일: `_sass/themes/_light.scss`

### One UI redesign

- `_posts` 서식 자동 정리 변경은 사용자 요청으로 롤백했다.
- 이후 디자인 방향을 One UI 8.5 계열로 다시 전환했다.
- 핵심 변경:
  - 배경을 순백에 가깝게 고정
  - 사이드바를 별도 surface 로 분리
  - 카드/패널/탑바를 같은 라운드/보더/쉐도우 체계로 통일
  - 라이트/다크 모두 같은 정보 계층 구조를 유지
