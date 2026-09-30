#!/usr/bin/env python3
"""Check source syntax, JSON, README length, and local Markdown links."""

from __future__ import annotations

import ast
import json
import re
import sys
from pathlib import Path
from urllib.parse import unquote, urlsplit


ROOT = Path(__file__).resolve().parents[1]
EXCLUDED = {".git", ".venv", "venv", "__pycache__", "logs", "tmp"}
LINK = re.compile(r"!?\[[^\]\n]*\]\((<[^>]+>|[^)\n]+)\)")


def repository_files() -> list[Path]:
    return sorted(
        path for path in ROOT.rglob("*")
        if path.is_file()
        and not (set(path.relative_to(ROOT).parts) & EXCLUDED)
    )


def local_links(path: Path, content: str) -> list[str]:
    errors: list[str] = []
    without_code = re.sub(r"```.*?```", "", content, flags=re.DOTALL)
    for match in LINK.finditer(without_code):
        target = match.group(1).strip()
        if target.startswith("<") and target.endswith(">"):
            target = target[1:-1]
        if urlsplit(target).scheme or target.startswith("#"):
            continue
        relative_target = unquote(target.split("#", 1)[0].split("?", 1)[0])
        if not relative_target:
            continue
        destination = (ROOT / relative_target.lstrip("/")) if relative_target.startswith("/") else (path.parent / relative_target)
        if not destination.exists():
            errors.append(f"{path.relative_to(ROOT)}: missing link target {target}")
    return errors


def main() -> int:
    errors: list[str] = []
    counts = {"python": 0, "json": 0, "markdown": 0}
    for path in repository_files():
        suffix = path.suffix.lower()
        if suffix not in {".py", ".json", ".md"}:
            continue
        try:
            content = path.read_text(encoding="utf-8-sig")
            if suffix == ".py":
                ast.parse(content, filename=str(path))
                compile(content, str(path), "exec", dont_inherit=True)
                counts["python"] += 1
            elif suffix == ".json":
                json.loads(content)
                counts["json"] += 1
            else:
                errors.extend(local_links(path, content))
                counts["markdown"] += 1
        except (OSError, UnicodeError, SyntaxError, ValueError) as exc:
            errors.append(f"{path.relative_to(ROOT)}: {exc}")

    readme = ROOT / "README.md"
    if not readme.exists():
        errors.append("README.md is missing")
    else:
        length = len(readme.read_text(encoding="utf-8"))
        print(f"README characters: {length}")
        if length < 5000:
            errors.append("README.md must contain at least 5000 characters")

    print("Checked: " + ", ".join(f"{value} {key}" for key, value in counts.items()))
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print("Repository checks passed. No robot hardware was accessed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
