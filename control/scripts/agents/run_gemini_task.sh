#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <worktree-path>"
  exit 1
fi

cd "$1"

if command -v gemini >/dev/null 2>&1; then
  exec gemini
else
  echo "gemini CLI not found. Install it, then rerun."
  exit 2
fi
