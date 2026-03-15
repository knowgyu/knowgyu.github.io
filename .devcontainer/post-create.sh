#!/usr/bin/env bash

set -euo pipefail

append_if_missing() {
  local line="$1"
  local file="$2"

  if ! grep -Fqx "$line" "$file" 2>/dev/null; then
    printf '%s\n' "$line" >>"$file"
  fi
}

ZSHRC="${HOME}/.zshrc"
BASHRC="${HOME}/.bashrc"
NVM_DIR="/usr/local/share/nvm"
NPMRC="${HOME}/.npmrc"
NPM_GLOBAL_PREFIX="${HOME}/.npm-global"
NPM_GLOBAL_BIN="${NPM_GLOBAL_PREFIX}/bin"
CODEX_HOME_DIR="${CODEX_HOME:-${HOME}/.codex}"
CODEX_VERSION="0.114.0"

configure_shell_env() {
  local file="$1"

  if [ -f "$file" ]; then
    append_if_missing "" "$file"
    append_if_missing "# Codex CLI" "$file"
    append_if_missing "export CODEX_HOME=\"${CODEX_HOME_DIR}\"" "$file"
    append_if_missing "export PATH=\"${NPM_GLOBAL_BIN}:\$PATH\"" "$file"
  fi
}

remove_line_if_present() {
  local pattern="$1"
  local file="$2"

  if [ -f "$file" ]; then
    sed -i "\|${pattern}|d" "$file"
  fi
}

mkdir -p "${NPM_GLOBAL_BIN}" "${CODEX_HOME_DIR}"
export CODEX_HOME="${CODEX_HOME_DIR}"
export PATH="${NPM_GLOBAL_BIN}:${PATH}"

remove_line_if_present '^export NPM_CONFIG_PREFIX=' "$ZSHRC"
remove_line_if_present '^export NPM_CONFIG_PREFIX=' "$BASHRC"
remove_line_if_present '^prefix=' "$NPMRC"

configure_shell_env "$ZSHRC"
configure_shell_env "$BASHRC"

if [ -f Gemfile ]; then
  echo "Installing Ruby gems..."
  bundle install
fi

if [ -s "${NVM_DIR}/nvm.sh" ]; then
  echo "Installing Node.js LTS with nvm..."
  set +u
  # shellcheck disable=SC1091
  . "${NVM_DIR}/nvm.sh"
  nvm install --lts
  nvm use --lts
  set -u
  export PATH="${NPM_GLOBAL_BIN}:${PATH}"
  node --version
  npm --version

  if [ -f package.json ]; then
    echo "Found package.json, installing npm dependencies..."
    npm install
    npm run build --if-present
  fi

  if ! command -v codex >/dev/null 2>&1; then
    echo "Installing Codex CLI..."
    npm install -g --prefix "${NPM_GLOBAL_PREFIX}" "@openai/codex@${CODEX_VERSION}" && codex --version || echo "Skipping Codex CLI install."
  fi

  if ! command -v netlify >/dev/null 2>&1; then
    echo "Installing Netlify CLI..."
    npm install -g --prefix "${NPM_GLOBAL_PREFIX}" netlify-cli && netlify --version || echo "Skipping Netlify CLI install."
  fi

  if [ -f "$ZSHRC" ]; then
    append_if_missing "# NVM auto load" "$ZSHRC"
    append_if_missing "export NVM_DIR=\"${NVM_DIR}\"" "$ZSHRC"
    append_if_missing "[ -s \"\$NVM_DIR/nvm.sh\" ] && . \"\$NVM_DIR/nvm.sh\"" "$ZSHRC"
  fi

  if [ -f "$BASHRC" ]; then
    append_if_missing "# NVM auto load" "$BASHRC"
    append_if_missing "export NVM_DIR=\"${NVM_DIR}\"" "$BASHRC"
    append_if_missing "[ -s \"\$NVM_DIR/nvm.sh\" ] && . \"\$NVM_DIR/nvm.sh\"" "$BASHRC"
  fi
else
  echo "Skipping Node.js setup because nvm was not found at ${NVM_DIR}."
fi

if command -v curl >/dev/null 2>&1; then
  echo "Installing shfmt via webi..."
  curl -fsSL https://webi.sh/shfmt | sh >/dev/null 2>&1 || echo "Skipping shfmt install."
fi

if [ -d "${HOME}/.oh-my-zsh/custom/plugins" ]; then
  echo "Installing zsh plugins..."

  if [ ! -d "${HOME}/.oh-my-zsh/custom/plugins/zsh-syntax-highlighting" ]; then
    git clone https://github.com/zsh-users/zsh-syntax-highlighting.git "${HOME}/.oh-my-zsh/custom/plugins/zsh-syntax-highlighting" || echo "Skipping zsh-syntax-highlighting install."
  fi

  if [ ! -d "${HOME}/.oh-my-zsh/custom/plugins/zsh-autosuggestions" ]; then
    git clone https://github.com/zsh-users/zsh-autosuggestions "${HOME}/.oh-my-zsh/custom/plugins/zsh-autosuggestions" || echo "Skipping zsh-autosuggestions install."
  fi

  if [ -f "$ZSHRC" ]; then
    if ! grep -Eq '^plugins=\(.*zsh-syntax-highlighting.*zsh-autosuggestions.*\)$' "$ZSHRC"; then
      sed -i -E 's/^plugins=\((.*)\)$/plugins=(\1 zsh-syntax-highlighting zsh-autosuggestions)/' "$ZSHRC"
    fi
    append_if_missing "" "$ZSHRC"
    append_if_missing "unset LESS" "$ZSHRC"
  fi
fi

if command -v sudo >/dev/null 2>&1; then
  echo "Installing Python packages..."
  sudo apt-get update && sudo apt-get install -y python-is-python3 python3-pip && python3 -m pip install --user pyyaml python-dotenv openai numpy || echo "Skipping Python package install."
fi
