from pybricks.hubs import PrimeHub

hub = PrimeHub()

# This file is used to save a password to the hub to be used for encryption.
# Make sure to only populate this string when writing to the hub and delete it when done.
# Do NOT commit this file with a real password.
# Password must be a number between 0 and 255
key = -1

if not isinstance(key, int):
    raise TypeError("Key must be an integer")
if not 0 <= key <= 255:
    raise ValueError("Key must be between 0 and 255")


key_bytes = key.to_bytes(1, "big")
hub.system.storage(0, write=key_bytes)