#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0
"""Convert main/include/maps.h C++ PROGMEM 2-D arrays into Rust const tables.

Run from repo root:
    python transictrl/tools/maps-converter/convert_maps.py \
        main/include/maps.h \
        transictrl/crates/transictrl-722/src/generated_maps.rs
"""

import re
import sys
from pathlib import Path


ARRAY_START_RE = re.compile(
    r"static\s+const\s+int\s+(\w+)\[(\d+)\]\[(\d+)\]\s+PROGMEM\s*\{",
)


def extract_braced(text: str, start: int) -> str:
    """Return the content between the brace at start and its matching close brace."""
    depth = 0
    i = start
    while i < len(text):
        c = text[i]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return text[start + 1 : i]
        i += 1
    return text[start + 1 :]


def parse_values(body: str) -> list[list[int]]:
    rows = []
    for line in body.splitlines():
        line = line.split("//")[0].strip()
        if not line:
            continue
        nums = re.findall(r"-?\d+", line)
        if nums:
            rows.append([int(n) for n in nums])
    return rows


def rust_array(name: str, rows: list[list[int]]) -> str:
    cols = len(rows[0]) if rows else 0
    inner = ",\n        ".join(
        "[" + ", ".join(f"{v:4d}" for v in row) + "]" for row in rows
    )
    return (
        f"pub const {name.upper()}: &[[i16; {cols}]] = &[\n"
        f"        {inner},\n"
        f"    ];"
    )


def convert(src: Path, dst: Path) -> None:
    text = src.read_text(encoding="utf-8")
    blocks = []
    pos = 0
    while True:
        m = ARRAY_START_RE.search(text, pos)
        if not m:
            break
        name, _rows, _cols = m.groups()
        body = extract_braced(text, m.end() - 1)
        rows = parse_values(body)
        pos = m.end() + len(body) + 1
        if not rows:
            continue
        # Skip the bitmap logo.
        if name == "mersu_map":
            continue
        blocks.append(rust_array(name, rows))

    # If a name appears twice (e.g. a commented-out table followed by an active
    # one with the same name), keep the last occurrence.
    seen = {}
    for block in blocks:
        name = block.split(":", 1)[0].split()[-1]
        seen[name] = block
    deduped = list(seen.values())

    header = (
        "// SPDX-License-Identifier: GPL-3.0-only\n"
        "// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0\n"
        "\n"
        "// Auto-generated from `main/include/maps.h`.\n"
        "//\n"
        "// Do not edit by hand. Re-run `transictrl/tools/maps-converter/convert_maps.py`.\n"
        "\n"
    )
    dst.write_text(header + "\n".join(deduped) + "\n", encoding="utf-8")
    print(f"Wrote {len(deduped)} tables to {dst}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <maps.h> <out.rs>")
        sys.exit(1)
    convert(Path(sys.argv[1]), Path(sys.argv[2]))
