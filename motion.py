"""Shared motion helpers for Spike Soccer robots.

Provides a `move` helper and the `QUADRANT_FUNCS` lookup table.  The
`move` helper accepts a direction in degrees, a speed and a tuple of four
motor ports (or motor objects).  The order of the tuple determines the
orientation so callers can supply their motor ports in whatever order
matches their physical build.
"""

from __future__ import annotations
from typing import Iterable, Sequence

try:  # Optional MicroPython modules may not be available when testing
    import motor as _motor_module
except Exception:  # pragma: no cover - best effort for missing deps
    _motor_module = None

# Inputs: quadrant (0-3) and ratio (0-2)
# Quadrant: the sector of the full 360 degree circle in which the direction lies.
# Ratio: the position within that quadrant, where 0 is the start and 2 is the end.
# Outputs: a multiplier for each of the four motors (-1 to 1).
QUADRANT_FUNCS = [
    lambda r: (r - 1, 1, -1, 1 - r),
    lambda r: (1, 1 - r, r - 1, -1),
    lambda r: (1 - r, -1, 1, r - 1),
    lambda r: (-1, r - 1, 1 - r, 1),
]


def move(direction: int, speed: int, motor_ports: Sequence):
    """Drive robot toward ``direction`` at ``speed``.

    Args:
        direction: Desired travel direction in degrees.
        speed: Motor speed (0-1110).
        motor_ports: Iterable of four motor objects or port identifiers.
            If an element has a ``run`` method it will be called with the
            computed speed.  Otherwise, if the ``motor`` module is
            available, ``motor.run(port, speed)`` will be used.
    """
    # --- Lookup table for octant vectors ---
    octant = (direction % 360) // 90
    ratio = (direction % 90) / 45
    a_mult, b_mult, c_mult, d_mult = QUADRANT_FUNCS[octant](ratio)
    for port, mult in zip(motor_ports, (a_mult, b_mult, c_mult, d_mult)):
        value = int(mult * speed)
        if hasattr(port, "run"):
            port.run(value)
        elif _motor_module is not None:
            _motor_module.run(port, value)
        else:  # pragma: no cover - unexpected scenario
            raise TypeError("motor port has no run() method and motor module unavailable")
