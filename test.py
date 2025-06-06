from hub import light_matrix, port
import runloop
import motor

OCTANT_FUNCS = [
    lambda r: (r-1, 1, -1, 1-r),    # 0°‑44°N → NE
    lambda r: (r, 1, -1, -r),    # 45°‑89° NE → E
    lambda r: (1, 1-r, r-1, -1),    # 90°‑134° E → SE
    lambda r: (1, -r, r, -1),    # 135°‑179° SE → S
    lambda r: (1-r, -1, 1, r-1),    # 180°‑224° S → SW
    lambda r: (-r, -1, 1, r),    # 225°‑269° SW → W
    lambda r: (-1, -(1-r), 1-r, 1), # 270°‑314° W → NW
    lambda r: (-1, r, -r, 1)        # 315°‑359° NW → N
]

# ---------------------------------------------
# Motor helper
# ---------------------------------------------

def move(direction: int, speed: int):
    """Drive robot toward `direction` (degrees) at `speed` (0-1110)."""

    # --- Lookup table for octant vectors ---
    octant = (direction % 360) // 45
    ratio = (direction % 45) / 45
    a_mult, b_mult, c_mult, d_mult = OCTANT_FUNCS[octant](ratio)

    motor.run(port.A, int(a_mult * speed))
    motor.run(port.B, int(b_mult * speed))
    motor.run(port.C, int(c_mult * speed))
    motor.run(port.D, int(d_mult * speed))

async def main():
  while True:
    move(180, 500)

runloop.run(main())
