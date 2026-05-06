#!/usr/bin/env python3
"""Normalize markdown tables to a simple pipe-delimited style."""

from __future__ import annotations

import argparse
from pathlib import Path


def normalize_table(lines: list[str]) -> list[str]:
    normalized: list[str] = []
    for line in lines:
        if "|" not in line:
            normalized.append(line)
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        normalized.append("| " + " | ".join(cells) + " |\n")
    return normalized


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("file", type=Path)
    args = parser.parse_args()

    content = args.file.read_text(encoding="utf-8").splitlines(keepends=True)
    args.file.write_text("".join(normalize_table(content)), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
