#!/bin/bash

set -euo pipefail

ROOT_DIR="/workspaces/knowgyu.github.io"
FUNCTIONS_DIR="$ROOT_DIR/knowgyu-ai-functions"

if [ "$#" -gt 0 ] && [[ "$1" =~ ^(-h|--help)$ ]]; then
  cat <<'EOF'
Usage: ./embedding_scripts.sh [--dry-run]

Required env:
  OPENAI_API_KEY
  SUPABASE_URL
  SUPABASE_SERVICE_ROLE_KEY

Optional env:
  OPENAI_EMBEDDING_MODEL
  SUPABASE_CHUNKS_TABLE
  CHUNK_TARGET_CHARACTERS
  CHUNK_OVERLAP_CHARACTERS
  SUPABASE_SYNC_BATCH_SIZE
EOF
  exit 0
fi

cd "$FUNCTIONS_DIR"
npm run sync:supabase -- "$@"
