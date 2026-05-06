#!/usr/bin/env bash
set -euo pipefail

root_dir="$(git rev-parse --show-toplevel)"

if [[ $# -eq 0 ]]; then
  echo "Usage: $0 <worktree-path> [worktree-path ...]"
  echo "Example: $0 .agents/task-1 .agents/task-2"
  exit 1
fi

for path in "$@"; do
  if [[ -d "$path" ]]; then
    git -C "$root_dir" worktree remove "$path"
    echo "Removed worktree: $path"
  else
    echo "Skipped (not found): $path"
  fi
done

git -C "$root_dir" worktree prune
