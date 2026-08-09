# knowgyu.github.io

This repository is a Korean technical archive and portfolio built with Jekyll/Chirpy.

## Canonical documents

Read the relevant document before editing:

- [README.md](README.md): repository purpose, local development, verification, and GitHub Pages deployment.
- [DESIGN.md](DESIGN.md): visual, accessibility, and route contracts.
- [docs/WRITING.md](docs/WRITING.md): post roles, front matter, and editorial templates.

`.omx/` is ignored runtime state. Its plans, logs, screenshots, and drafts are not canonical editorial or engineering guidance.

## Content changes

- Keep one topical category path of at most two levels.
- Treat categories as topics, not project status or a dumping ground.
- Add `project:` only when a post belongs to one concrete project.
- Follow `docs/WRITING.md`; do not invent front matter fields or category branches without updating it.
- Preserve public post URLs and post bodies unless a change explicitly requires a migration.

## Engineering changes

- Preserve Jekyll/Chirpy, search, feeds, sitemap, responsive navigation, and light/dark mode.
- Do not add dependencies for editorial or layout work.
- Keep `_data/taxonomy.yml` as the curated taxonomy/navigation source.
- Run the narrowest relevant check first, then `bash tools/test.sh` for publishable changes. Run `bash tools/visual-qa.sh` for visual changes.

## Release

Pushes to `main` build and deploy GitHub Pages. A release is complete only after the workflow succeeds and the affected public routes return successfully.
