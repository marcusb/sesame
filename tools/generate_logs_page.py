#!/usr/bin/env python3
"""
Generates include/logs_page.h from src/logs_page.html
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HTML_SRC = ROOT / "src" / "logs_page.html"
HEADER_OUT = ROOT / "include" / "logs_page.h"


def main():
    if not HTML_SRC.exists():
        print(f"Error: {HTML_SRC} not found", file=sys.stderr)
        sys.exit(1)

    html = HTML_SRC.read_text(encoding="utf-8")

    lines = html.splitlines(keepends=True)
    c_lines = []
    c_lines.append(
        "/* Auto-generated from src/logs_page.html - DO NOT EDIT MANUALLY */\n"
    )
    c_lines.append("#ifndef LOGS_PAGE_H_\n")
    c_lines.append("#define LOGS_PAGE_H_\n\n")
    c_lines.append("static const char logs_html[] =\n")

    for line in lines:
        escaped = (
            line.replace("\\", "\\\\")
            .replace('"', '\\"')
            .replace("\r", "")
            .replace("\n", "\\n")
        )
        c_lines.append(f'"{escaped}"\n')

    c_lines.append(";\n\n")
    c_lines.append("#endif /* LOGS_PAGE_H_ */\n")

    HEADER_OUT.write_text("".join(c_lines), encoding="utf-8")
    print(f"Generated {HEADER_OUT} ({len(html)} bytes HTML)")


if __name__ == "__main__":
    main()
