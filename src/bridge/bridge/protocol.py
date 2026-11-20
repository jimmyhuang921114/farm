import json


def encode_packet(command, **data):
    packet = {
        "cmd": command,
        "data": data,
    }

    return (
        json.dumps(
            packet,
            separators=(",", ":"),
            allow_nan=False,
        ) + "\n"
    ).encode("utf-8")


    

class PacketParser:  #package
    def __init__(self): 
        self.buffer = bytearray() # create a array for buffer the msg
        self.max_packet_size = 1024 # max packet size of byte

    def feed(self, received: bytes) -> list[dict]: # get the receiver from callback
        self.buffer.extend(received) # extend  the receiver
        packets = []

        while b"\n" in self.buffer: 
            line, _, remaining = self.buffer.partition(b"\n")
            self.buffer = bytearray(remaining)

            if not line or len(line) > self.max_packet_size:
                continue

            try:
                packet = json.loads(line.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue

            if (
                isinstance(packet, dict)
                and isinstance(packet.get("cmd"), str)
                and isinstance(packet.get("data"), dict)
            ):
                packets.append(packet)

        # if the buffer longer than setting size,clear the buffer
        if len(self.buffer) > self.max_packet_size:
            self.buffer.clear()

        return packets