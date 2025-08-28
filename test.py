from hub import light_matrix, port
import runloop
import motor

from motion import move

async def main():
  while True:
    move(45, 500, (port.A, port.B, port.C, port.D))

runloop.run(main())
