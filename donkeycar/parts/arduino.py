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
            re.compile('(?P<timestamp>\d+),(?P<channel_1>\d+),(?P<channel_2>\d+),(?P<channel_3>\d+),(?P<channel_4>\d+),'
                       '(?P<channel_5>\d+),(?P<channel_6>\d+),(?P<frequency>\d+)')
        self._serial = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        self._process_channel_2 = None
        self._process_channel_1 = None
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

                if 'channel_1' in groups:
                    self._process_channel_1 = int(groups['channel_1'])
                if 'channel_2' in groups:
                    self._process_channel_2 = int(groups['channel_2'])
                if 'channel_3' in groups:
                    self._process_channel_3(int(groups['channel_3']))
                if 'channel_4' in groups:
                    self._process_channel_4(int(groups['channel_4']))
                if 'channel_5' in groups:
                    self._process_channel_5(int(groups['channel_5']))
                if 'channel_6' in groups:
                    self._process_channel_6(int(groups['channel_6']))
            except Exception as error:
                logging.exception("Unexpected error: %s", error)

    def _process_channel_3(self, value):
        pass

    def _process_channel_4(self, value):
        pass

    def _process_channel_5(self, value):
        pass

    def _process_channel_6(self, value):
        if value > 1800:
            if self._user_mode != DRIVE_MODE_PILOT:
                logger.info('Update channel 6 with value %s, new user_mode: %s', value, DRIVE_MODE_PILOT)
            self._user_mode = DRIVE_MODE_PILOT
        else:
            if self._user_mode != DRIVE_MODE_USER:
                logger.info('Update channel 6 with value %s, new user_mode: %s', value, DRIVE_MODE_USER)
            self._user_mode = DRIVE_MODE_USER

    def run_threaded(self) -> (int, int, str, bool):
        return self._process_channel_1, self._process_channel_2, self._user_mode

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
