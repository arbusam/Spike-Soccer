from pybricks.hubs import PrimeHub


def clear_error():
    """Overwrite the persisted error message with zeroed bytes."""
    hub = PrimeHub()
    offset = 1  # striker.py stores the error at byte offset 1
    zeros = bytes(500)  # 500 zeroed bytes

    try:
        hub.system.storage(offset, write=zeros)
    except ValueError:
        print("Unable to access storage.")
        return

    print("Error storage cleared.")


if __name__ == "__main__":
    clear_error()
