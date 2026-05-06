#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <worktree-path>"
  exit 1
fi

worktree_path="$1"
cd "$worktree_path"

export CODEX_HOME="${PWD}/.codex"
mkdir -p "$CODEX_HOME"

if command -v codex >/dev/null 2>&1; then
  exec codex
else
  echo "codex CLI not found. Install it, then rerun."
  exit 2
fi
