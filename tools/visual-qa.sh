#!/usr/bin/env bash
set -euo pipefail

# Uses the repository's existing Playwright/Chromium toolchain. No package or
# browser installation is performed by this runner.
module_root="${PLAYWRIGHT_NODE_MODULES:-}"
if [[ -z "$module_root" && -f "$PWD/node_modules/playwright/index.js" ]]; then
  module_root="$PWD/node_modules"
fi

if [[ -z "$module_root" && -n "${PLAYWRIGHT_BIN:-}" ]]; then
  cli_path="$(readlink -f "$PLAYWRIGHT_BIN")"
  module_root="$(dirname "$(dirname "$cli_path")")"
fi

if [[ -z "$module_root" && -d "${HOME:-}/.npm/_npx" ]]; then
  for candidate in "${HOME}/.npm/_npx"/*/node_modules; do
    if [[ -f "$candidate/playwright/index.js" ]]; then
      module_root="$candidate"
      break
    fi
  done
fi

if [[ -n "$module_root" && -f "$module_root/playwright/index.js" ]]; then
  export NODE_PATH="$module_root${NODE_PATH:+:$NODE_PATH}"
  exec node tests/visual-qa.spec.cjs "$@"
fi

echo 'Playwright is unavailable; set PLAYWRIGHT_NODE_MODULES to the existing node_modules path.' >&2
exit 127
