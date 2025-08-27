"""
Simple XOR encryption demo matching the robot code.

- Hardcodes a random key (0..255)
- Encrypts a message (str or int) to bytes (simulating BLE broadcast payload)
- "Receives" those bytes and decrypts back to the original type
- Prints the decrypted result

This file does not access any hardware or saved key.
"""
from __future__ import annotations

from typing import Union

# Hardcoded test key (0..255)
KEY: int = 173  # random byte value


def xor(data: Union[bytes, str, int], key: int) -> bytes:
    """XOR data with a single-byte key.

    - If data is str, it is XOR'ed per character code and returned as bytes.
    - If data is bytes, each byte is XOR'ed and returned as bytes.
    - If data is int, it is converted to a single byte then XOR'ed.
    """
    if not isinstance(key, int):
        raise TypeError("Key must be an integer")
    if not 0 <= key <= 255:
        raise ValueError("Key must be between 0 and 255")

    if isinstance(data, bytes):
        return bytes([b ^ key for b in data])
    elif isinstance(data, str):
        return bytes([ord(ch) ^ key for ch in data])
    elif isinstance(data, int):
        if not 0 <= data <= 255:
            raise ValueError("Int must be in range 0..255 to fit in one byte")
        return bytes([data ^ key])
    else:
        raise TypeError("Data must be bytes or str")

def encrypt(message: Union[str, int], key: int) -> bytes:
    """Encrypt a str or small int (0..255) to bytes using XOR(key).

    For ints, we add a 1-byte redundancy: [value, value ^ 0xFF] before XOR,
    then XOR both bytes to produce the payload. This lets the receiver validate.
    """
    if isinstance(message, int):
        if not 0 <= message <= 255:
            raise ValueError("Int must be in range 0..255 to fit in one byte")
        raw = bytes((message, message ^ 0xFF))
        return xor(raw, key)
    return xor(message, key)

def decrypt(payload: Union[bytes, None], key: int):
    """Decrypt XOR(key) bytes back to either str (only 'O' or 'T') or int.

    Returns None if payload is None or fails validation.
    Validation rules:
      - Strings must decode to exactly one of 'O' or 'T'.
      - Ints are taken from decrypted bytes (big-endian). Accept only if < 500.
    """
    if payload is None:
        return None
    if not isinstance(payload, (bytes, bytearray)):
        return None

    decrypted = xor(bytes(payload), key)

    # String path: only one byte messages 'O' or 'T' are valid.
    if len(decrypted) == 1:
        try:
            text = decrypted.decode("utf-8")
            if text in ("O", "T"):
                return text
        except Exception:
            pass
        return None

    # Int path: expect 2 bytes [value, value ^ 0xFF]
    if len(decrypted) >= 2:
        v0, v1 = decrypted[0], decrypted[1]
        if (v0 ^ v1) == 0xFF:
            return v0
        return None

    return None


def simulate_broadcast_and_receive(message: Union[str, int], key: int):
    """Simulate the striker broadcasting and the defence receiving.

    Returns the decrypted message that the receiver would see.
    """
    # Sender encrypts to bytes (what would be sent over BLE)
    payload = encrypt(message, key)

    # Receiver "observes" bytes and decrypts
    received = decrypt(payload, key)
    return received


if __name__ == "__main__":
    samples = [
        "Hello, Spike!",
        "T",  # matches your code's touch/hold signal
        "O",  # matches your code's own-goal prevention signal
        0,
        42,
        255,
    ]

    print(f"Using KEY={KEY}")
    for msg in samples:
        enc = encrypt(msg, KEY)
        dec = decrypt(enc, KEY)
        print("-" * 40)
        print(f"Plain: {msg!r}")
        print(f"Encrypted bytes: {enc}")
        print(f"Decrypted: {dec!r}")

    # Single end-to-end demo
    print("-" * 40)
    demo_message = 123
    result = simulate_broadcast_and_receive(demo_message, KEY)
    print(f"E2E result for {demo_message!r}: {result!r}")
