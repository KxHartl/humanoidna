#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <task-slug>"
  exit 1
fi

slug="$1"
root_dir="$(git rev-parse --show-toplevel)"
branch="task/${slug}"
worktree="${root_dir}/.agents/${slug}"

git -C "$root_dir" fetch --all --prune

if git -C "$root_dir" show-ref --verify --quiet "refs/heads/${branch}"; then
  git -C "$root_dir" worktree add "$worktree" "$branch"
else
  git -C "$root_dir" worktree add -b "$branch" "$worktree" main
fi

echo "Worktree created: $worktree"
echo "Branch: $branch"
