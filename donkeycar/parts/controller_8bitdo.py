import logging
import struct
from typing import List

from donkeycar.parts.arduino import DRIVE_MODE_PILOT, DRIVE_MODE_USER
from donkeycar.parts.mqtt import USER_MODE
from donkeycar.parts.part import ThreadedPart

SNES_CONTROLLER_ANGLE = 'snes_controller_angle'
SNES_CONTROLLER_THROTTLE = 'snes_controller_throttle'

logger = logging.getLogger(__name__)


class Snes8bitdoController(ThreadedPart):
    # long int, long int, unsigned short, unsigned short, unsigned int
    _FORMAT = 'llHHI'
    _EVENT_SIZE = struct.calcsize(_FORMAT)

    code_axis_left_right = 0
    code_axis_top_down = 1
    code_bt_a = 304
    code_bt_b = 305
    code_bt_x = 307
    code_bt_y = 308
    code_pad_left_ = 310
    code_pad_right = 311
    code_select = 314
    code_start = 315

    def __init__(self, infile_path='/dev/input/event0'):
        self._in_file = open(infile_path, "rb")
        self._mode = None
        self._throttle = 0
        self._angle = 0

    def update(self):
        event = self._in_file.read(__class__._EVENT_SIZE)
        while event:
            (tv_sec, tv_usec, type, code, value) = struct.unpack(__class__._FORMAT, event)

            if type != 0 or code != 0 or value != 0:
                logger.info("Event type %u, code %u, value %u", type, code, value)
                if code == 0 or code == 1:
                    self._update_axis_state(code, value)
                else:
                    self._update_button(code, value)
            event = self._in_file.read(__class__._EVENT_SIZE)

    def run_threaded(self, user_mode):
        self._mode = user_mode
        return self._mode, self._throttle, self._angle

    def get_inputs_keys(self) -> List[str]:
        return [USER_MODE]

    def get_outputs_keys(self) -> List[str]:
        return [USER_MODE, SNES_CONTROLLER_THROTTLE, SNES_CONTROLLER_ANGLE]

    def shutdown(self):
        self._in_file.close()

    def _update_axis_state(self, code, value):
        if value > 100:
            self._angle = 0.5
        elif value < -100:
            self._angle = -0.5
        else:
            self._angle = 0

    def _update_button(self, code, value):
        if code == __class__.code_bt_a:
            if value == 1:
                self._mode = DRIVE_MODE_PILOT
        if code == __class__.code_bt_y:
            if value == 1:
                self._mode = DRIVE_MODE_USER

        if code == __class__.code_bt_b:
            if value == 1:
                self._throttle = 0.5
            else:
                self._throttle = 0

        if code == __class__.code_bt_x:
            if value == 1:
                self._throttle = -0.5
            else:
                self._throttle = 0
