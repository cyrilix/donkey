import logging
import re
from typing import List

import serial

from donkeycar.parts.mqtt import USER_MODE
from donkeycar.parts.part import ThreadedPart

DRIVE_MODE_USER = 'user'
DRIVE_MODE_PILOT = 'local'
DRIVE_MODE_LOCAL_ANGLE = 'local_angle'

SHOCK = 'shock'

PWM_THROTTLE = 'pwm/throttle'
PWM_STEERING = 'pwm/steering'

logger = logging.getLogger(__name__)


class SerialPart(ThreadedPart):

    def __init__(self, port="/dev/ttyS0", baudrate=115200):
        self._line_regex = \
            re.compile('(?P<timestamp>\d+),(?P<steering>\d+),(?P<throttle>\d+),(?P<user_mode>\d+),(?P<frequency>\d+)')
        self._serial = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        self._throttle_pwm = None
        self._steering_pwm = None
        self._user_mode = 'user'
        self._on = True
        self._button_is_pushed = False

    def update(self):
        logger.info("Start SerialPart")
        while self._on:
            try:
                line = self._serial.readline().decode("utf-8")
                match = self._line_regex.match(line)
                if not match:
                    continue
                groups = match.groupdict()

                if 'steering' in groups:
                    self._steering_pwm = int(groups['steering'])
                if 'throttle' in groups:
                    self._throttle_pwm = int(groups['throttle'])
                if 'user_mode' in groups:
                    self._process_channel3(int(groups['user_mode']))
            except:
                logging.exception("Unexpected error")

    def _process_channel3(self, value):
        if not self._button_is_pushed and value > 1500:
            self._button_is_pushed = True
            if self._user_mode == DRIVE_MODE_USER:
                self._user_mode = DRIVE_MODE_PILOT
            elif self._user_mode == DRIVE_MODE_PILOT:
                self._user_mode = DRIVE_MODE_USER
        if self._button_is_pushed and value < 1500:
            self._button_is_pushed = False

    def run_threaded(self) -> (int, int, str, bool):
        return self._steering_pwm, self._throttle_pwm, self._user_mode

    def get_inputs_keys(self) -> List[str]:
        return []

    def get_outputs_keys(self) -> List[str]:
        return [PWM_STEERING,
                PWM_THROTTLE,
                USER_MODE
                ]

    def shutdown(self):
        logger.info("Stop SerialPart")
        self._on = False
        self._serial.close()

    def run(self, **kw):
        pass
