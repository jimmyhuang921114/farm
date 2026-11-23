"""Write complete packets to the shared serial port."""


def send_packet(port, packet):
    """Write one encoded packet or raise an error on a short write."""
    if port.write(packet) != len(packet):
        raise OSError('Incomplete UART packet write')
