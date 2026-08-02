"""Small timing helpers for command-line reporting."""

from __future__ import annotations


def format_duration(seconds: float) -> str:
    seconds = float(seconds)
    if seconds < 1.0:
        return "{:.1f} ms".format(seconds * 1000.0)
    return "{:.3f} s".format(seconds)
