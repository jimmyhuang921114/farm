# api.py
from collections.abc import Callable
from copy import deepcopy

from .protocol import encode_packet


class RobotAPI: # the code of api
    def __init__(self, send: Callable[[bytes], None]):
        self._send = send #create the variable to save the msg

        # get the feedback from other code
        self._feedback = { 
            "pose": None,
            "wheel_speed": None,
            "pwm": None,
            "status": None,
        }

    # setting the target velocity
    def set_wheel_speed(self, left: float, right: float):
        self._send(
            encode_packet( # packet msg 
                "set_wheel_speed",
                left=float(left),
                right=float(right),
            )
        )

    def set_pwm(self, left_us: int, right_us: int):
        self._send(
            encode_packet(
                "set_pwm",
                left_us=int(left_us),
                right_us=int(right_us),
            )
        )

    def send_person_position(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
    ):
        self._send(
            encode_packet(
                "person_position",
                x=float(x),
                y=float(y),
                z=float(z),
            )
        )

    def send_person_lost(self):
        self._send(encode_packet("person_lost"))

    def stop(self): # stop sending msg
        self._send(encode_packet("stop"))

    def get_pose(self): # get the pose from stm32
        return deepcopy(self._feedback["pose"])

    def get_wheel_speed(self): # get the wheel actual speed
        return deepcopy(self._feedback["wheel_speed"])

    def get_pwm(self): # get the stm32 pwm
        return deepcopy(self._feedback["pwm"])

    def get_status(self): # get the stm32 status
        return deepcopy(self._feedback["status"])

    # update from stm32
    def update_feedback(self, packet: dict):
        name = packet["cmd"]

        if name in self._feedback:
            self._feedback[name] = deepcopy(packet["data"])