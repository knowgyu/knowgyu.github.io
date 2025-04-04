#!/bin/bash

# 임베딩 생성 및 Netlify 함수 서브모듈로 복사하는 스크립트

# 색상 정의
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 기본 경로 설정
MAIN_REPO="/workspaces/knowgyu.github.io"
NETLIFY_REPO="$MAIN_REPO/knowgyu-ai-functions"
EMBEDDINGS_SRC="$MAIN_REPO/assets/js/chatbot"
EMBEDDINGS_DEST="$NETLIFY_REPO/netlify/functions"

# 1. 현재 디렉토리 확인
if [ "$(pwd)" != "$MAIN_REPO" ]; then
  echo -e "${YELLOW}메인 레포지토리로 이동합니다...${NC}"
  cd "$MAIN_REPO" || {
    echo -e "${RED}메인 레포지토리로 이동할 수 없습니다!${NC}"
    exit 1
  }
fi

# 2. 임베딩 생성 스크립트 실행
echo -e "${GREEN}임베딩 생성 스크립트 실행 중...${NC}"
python3 assets/js/chatbot/prepare_data.py
if [ $? -ne 0 ]; then
  echo -e "${RED}임베딩 생성 실패!${NC}"
  exit 1
fi

# 3. Netlify 함수 레포지토리 존재 확인
if [ ! -d "$NETLIFY_REPO" ]; then
  echo -e "${RED}Netlify 함수 레포지토리를 찾을 수 없습니다!${NC}"
  exit 1
fi

# 4. 대상 디렉토리 생성
echo -e "${GREEN}Netlify 레포지토리에 대상 디렉토리 생성 중...${NC}"
mkdir -p "$EMBEDDINGS_DEST/embeddings"

# 5. 파일 복사
echo -e "${GREEN}임베딩 파일 복사 중...${NC}"

# posts-data.json 복사
cp "$EMBEDDINGS_SRC/posts-data.json" "$EMBEDDINGS_DEST/"
echo "posts-data.json 복사 완료"

# embeddings-index.js 복사
cp "$EMBEDDINGS_SRC/embeddings-index.js" "$EMBEDDINGS_DEST/"
echo "embeddings-index.js 복사 완료"

# 임베딩 디렉토리 내 파일 복사
cp -r "$EMBEDDINGS_SRC/embeddings/"* "$EMBEDDINGS_DEST/embeddings/"
echo "임베딩 파일 복사 완료"

# 6. 복사된 파일 수 확인
COPIED_FILES=$(find "$EMBEDDINGS_DEST/embeddings" -type f | wc -l)
TOTAL_FILES=$((COPIED_FILES + 2)) # embeddings-index.js와 posts-data.json 포함
echo -e "${GREEN}총 $TOTAL_FILES 개의 파일이 복사되었습니다.${NC}"

# 7. 선택적: Git 커밋 및 푸시
read -p "변경사항을 Git에 커밋하시겠습니까? (y/n): " GIT_CONFIRM
if [ "$GIT_CONFIRM" = "y" ] || [ "$GIT_CONFIRM" = "Y" ]; then
  echo -e "${YELLOW}Netlify 레포지토리의 변경사항을 커밋합니다...${NC}"
  cd "$NETLIFY_REPO" || exit
  git add .
  git commit -m "Update embeddings $(date '+%Y-%m-%d %H:%M:%S')"

  read -p "변경사항을 원격 저장소에 푸시하시겠습니까? (y/n): " PUSH_CONFIRM
  if [ "$PUSH_CONFIRM" = "y" ] || [ "$PUSH_CONFIRM" = "Y" ]; then
    git push
    echo -e "${GREEN}변경사항이 푸시되었습니다.${NC}"
  else
    echo -e "${YELLOW}변경사항이 로컬에만 커밋되었습니다.${NC}"
  fi
else
  echo -e "${YELLOW}변경사항이 커밋되지 않았습니다.${NC}"
fi

echo -e "${GREEN}작업 완료!${NC}"
