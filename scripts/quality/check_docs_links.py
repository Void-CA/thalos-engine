#!/usr/bin/env python3
"""Check relative Markdown links under docs/ (Thalos Engine)."""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DOCS = ROOT / "docs"

LINK = re.compile(r"\[[^\]]*\]\(([^)]+)\)")
FENCE = re.compile(r"```.*?```", re.S)


def main() -> int:
    broken: list[str] = []
    for path in sorted(DOCS.rglob("*.md")):
        text = FENCE.sub("", path.read_text(encoding="utf-8", errors="ignore"))
        for match in LINK.finditer(text):
            target = match.group(1).split("#", 1)[0].strip()
            if not target or target.startswith(("http://", "https://", "mailto:")):
                continue
            if not (path.parent / target).resolve().exists():
                broken.append(f"{path.relative_to(ROOT)}: {target}")

    if broken:
        print("Broken relative links:")
        for item in broken:
            print(f"  {item}")
        return 1

    print("docs links: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
