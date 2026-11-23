"""Encode STM32 commands and decode velocity replies."""

import json
import math


def _encode(command, data):
    return (
        json.dumps({'cmd': command, 'data': data}, allow_nan=False) + '\n'
    ).encode('ascii')


def encode_person_position(x, y, z):
    """Encode a person position in the robot base frame, in meters."""
    return _encode('person_position', {'x': x, 'y': y, 'z': z})


def encode_person_lost():
    """Encode a notification that the target is no longer available."""
    return _encode('person_lost', {})


def encode_stop():
    """Encode an explicit stop command."""
    return _encode('stop', {})


def decode_cmd_vel(line):
    """Return finite (v, w) values from a valid STM32 reply, or None."""
    try:
        packet = json.loads(line)
        if not isinstance(packet, dict) or packet.get('cmd') != 'cmd_vel':
            return None
        data = packet.get('data')
        if not isinstance(data, dict):
            return None
        v, w = data.get('v'), data.get('w')
        if all(type(value) in (int, float) and math.isfinite(value)
               for value in (v, w)):
            return float(v), float(w)
    except (ValueError, OverflowError):
        pass
    return None
