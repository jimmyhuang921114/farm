"""Assemble newline-delimited UART replies across serial reads."""


class UartReceiver:
    """Read complete lines without blocking the ROS executor."""

    def __init__(self, port):
        """Use the serial port opened by the bridge node."""
        self._port = port
        self._buffer = bytearray()
        self._discarding = False

    def read_lines(self):
        """Yield complete lines and discard oversized frames."""
        for byte in self._port.read(min(self._port.in_waiting, 4096)):
            if byte == 10:
                if not self._discarding and self._buffer:
                    yield bytes(self._buffer)
                self._buffer.clear()
                self._discarding = False
            elif not self._discarding:
                if len(self._buffer) < 256:
                    self._buffer.append(byte)
                else:
                    self._buffer.clear()
                    self._discarding = True
