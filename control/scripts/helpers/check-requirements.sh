#!/usr/bin/env bash
set -euo pipefail

root_dir="$(git rev-parse --show-toplevel)"
manifest="${root_dir}/config/requirements.list"

if [[ ! -f "$manifest" ]]; then
  echo "Requirement manifest not found: $manifest"
  exit 2
fi

status=0
printf '%-10s %-14s %-10s %s\n' "SCOPE" "NAME" "STATUS" "DETAILS"

while IFS='|' read -r scope name command required min_version install_hint notes; do
  [[ -z "${scope// }" ]] && continue
  [[ "$scope" == \#* ]] && continue

  if command -v "$command" >/dev/null 2>&1; then
    printf '%-10s %-14s %-10s %s\n' "$scope" "$name" "OK" "$notes"
  else
    printf '%-10s %-14s %-10s %s (%s)\n' "$scope" "$name" "MISSING" "$notes" "$install_hint"
    if [[ "$required" != "optional" && "$required" != "recommended" ]]; then
      status=1
    fi
  fi
done < "$manifest"

exit "$status"
