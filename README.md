# knowgyu.github.io

한국어 기술 아카이브이자 프로젝트 포트폴리오입니다. 로보틱스, 컴퓨터 비전, 로컬 검색, 에이전트 시스템을 만들며 공부한 내용·진행 기록·재사용 가능한 판단을 남깁니다.

## 정본 문서

- [DESIGN.md](DESIGN.md): UI·접근성·경로 계약
- [docs/WRITING.md](docs/WRITING.md): 콘텐츠 모델, front matter, 글 템플릿
- [AGENTS.md](AGENTS.md): 저장소 수정 시 지켜야 할 운영 규칙

## 구조

- `_posts/`: 공개 글
- `_tabs/`: Home, 전체 글, Projects 같은 상위 탐색면
- `_data/taxonomy.yml`: 사이드바와 Home의 주제 분류·표시 순서
- `_sass/`, `_includes/`, `_layouts/`: Chirpy 위의 편집형 블로그 UI
- `tools/`, `tests/`: 정적 계약, 빌드, Playwright 시각 검증

## 로컬 실행과 검증

처음 한 번 의존성을 설치한 뒤 서버를 실행합니다.

```bash
bundle install
bundle exec jekyll serve
```

공개 사이트에 영향을 주는 변경은 다음 전체 검증을 통과해야 합니다.

```bash
bash tools/test.sh
```

시각 변경은 빌드된 `_site/`를 제공한 뒤 실행합니다.

```bash
PLAYWRIGHT_NODE_MODULES=/path/to/node_modules \
  BASE_URL=http://127.0.0.1:4000 \
  bash tools/visual-qa.sh
```

## 배포

`main`의 사이트 소스 변경은 GitHub Actions에서 Jekyll 빌드·HTML 검사를 거쳐 GitHub Pages에 배포됩니다. 완료 기준은 Actions 성공과 `https://knowgyu.github.io/`의 영향을 받은 경로가 실제로 열리는 것입니다.

## 런타임 파일

`.omx/`는 로컬 작업 런타임·캐시·검증 산출물이며 Git 정본이 아닙니다. 제품·글·배포 규칙은 위 문서에서만 관리합니다.
