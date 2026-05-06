#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <input.md|input.tex> <output.docx>"
  exit 1
fi

input_file="$1"
output_file="$2"

if ! command -v pandoc >/dev/null 2>&1; then
  echo "Error: pandoc is required for DOCX export."
  exit 2
fi

pandoc "$input_file" -o "$output_file"
echo "Exported $output_file"
