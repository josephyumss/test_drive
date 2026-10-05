"""CRC-16/CCITT-FALSE implementation shared by packet tests."""


def crc16_ccitt_false(data: bytes) -> int:
    """Return CRC-16/CCITT-FALSE for *data*.

    Parameters: poly=0x1021, init=0xFFFF, refin=false, refout=false, xorout=0.
    """

    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc
