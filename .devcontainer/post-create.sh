#!/usr/bin/env bash

# 패키지 존재 여부에 관계없이 항상 NVM과 Node.js LTS 설치
echo "Installing NVM and Node.js LTS..."

# NVM 경로 설정
export NVM_DIR="/usr/local/share/nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"

# NVM 로드 확인
command -v nvm

nvm install --lts
nvm use --lts
node --version
npm --version

# Netlify CLI 전역 설치
echo "Installing Netlify CLI..."
npm install -g netlify-cli
netlify --version

# 기존 package.json 관련 로직도 유지
if [ -f package.json ]; then
  echo "Found package.json, installing dependencies..."
  npm i
  npm run build
fi

# Install dependencies for shfmt extension
curl -sS https://webi.sh/shfmt | sh &>/dev/null

# Add OMZ plugins
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ~/.oh-my-zsh/custom/plugins/zsh-syntax-highlighting
git clone https://github.com/zsh-users/zsh-autosuggestions ~/.oh-my-zsh/custom/plugins/zsh-autosuggestions
sed -i -E "s/^(plugins=\()(git)(\))/\1\2 zsh-syntax-highlighting zsh-autosuggestions\3/" ~/.zshrc

# Avoid git log use less
echo -e "\nunset LESS" >>~/.zshrc

# NVM 자동 로드 설정 추가
echo -e "\n# NVM 자동 로드\nexport NVM_DIR=\"\$HOME/.nvm\"\n[ -s \"\$NVM_DIR/nvm.sh\" ] && . \"\$NVM_DIR/nvm.sh\"" >>~/.zshrc
