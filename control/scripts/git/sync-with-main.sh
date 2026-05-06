#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <worktree-path>"
  exit 1
fi

worktree_path="$1"
root_dir="$(git rev-parse --show-toplevel)"

git -C "$root_dir" fetch origin
git -C "$worktree_path" rebase origin/main
echo "Synced $worktree_path with origin/main"
