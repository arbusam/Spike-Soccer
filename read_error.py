from pybricks.hubs import PrimeHub


def read_error():
    """Read the persisted error message written by striker.py and print it."""
    hub = PrimeHub()
    offset = 1  # striker.py stores the error at byte offset 1
    read_length = 512 - offset  # PrimeHub user storage capacity is 512 bytes

    try:
        raw = hub.system.storage(offset, read=read_length)
    except ValueError:
        print("Unable to access storage.")
        return

    stored = raw.split(b"\x00", 1)[0]
    try:
        message = stored.decode("utf-8")
    except (AttributeError, TypeError):
        # Fallback for MicroPython builds without bytes.decode.
        message = ""
        for byte in stored:
            if byte < 32 and byte not in (9, 10, 13):
                continue
            try:
                message += chr(byte)
            except ValueError:
                message += "?"
    message = message.strip()
    if message:
        print(message)
    else:
        print("No error stored.")


if __name__ == "__main__":
    read_error()
