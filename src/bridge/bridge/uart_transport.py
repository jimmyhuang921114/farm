import serial


class UARTTransport:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        self.uart = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.1,
            write_timeout=1.0,
        )

    def send(self, data: bytes):
        written = self.uart.write(data)

        if written != len(data):
            raise IOError("UART 封包未完整傳送")


    def read_available(self) -> bytes:
        count = self.uart.in_waiting

        if count == 0:
            return b""

        return self.uart.read(count)


    
    def close(self):
        try:
            self.uart.flush()
        finally:
            self.uart.close()