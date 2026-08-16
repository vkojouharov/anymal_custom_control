from __future__ import annotations

import sys
import time


WIDTH = 104


class Rate:
    def __init__(self):
        self.reset()

    def reset(self, now: float | None = None) -> None:
        self.hz = 0.0
        self._start = time.monotonic() if now is None else now
        self._count = 0

    def tick(self, now: float | None = None) -> float:
        now = time.monotonic() if now is None else now
        self._count += 1
        elapsed = now - self._start
        if elapsed >= 1.0:
            self.hz = self._count / elapsed
            self._count = 0
            self._start = now
        return self.hz


def vector(values, digits=2) -> str:
    return "[" + ", ".join(f"{float(value):+.{digits}f}" for value in values) + "]"


def tags(snapshot) -> str:
    if not snapshot.visible_tags:
        return "none"
    return " | ".join(
        f"ID {tag_id} xyz={vector(tag.T_camera_tag[:3, 3])}"
        for tag_id, tag in sorted(snapshot.visible_tags.items())
    )


def render(rows) -> None:
    label_width = 14
    value_width = WIDTH - label_width - 7
    border = "+" + "-" * (WIDTH - 2) + "+"
    lines = [border, f"|{'CABLE OUTFITTING':^{WIDTH - 2}}|", border]
    for label, value in rows:
        value = str(value)
        if len(value) > value_width:
            value = value[: value_width - 3] + "..."
        lines.append(f"| {label:<{label_width}} | {value:<{value_width}} |")
    lines.append(border)
    panel = "\n".join(lines)
    if sys.stdout.isatty():
        sys.stdout.write("\033[2J\033[H" + panel + "\n")
        sys.stdout.flush()
    else:
        print("  ".join(f"{label}={value}" for label, value in rows))
