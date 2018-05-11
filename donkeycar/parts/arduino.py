import logging
from typing import List

import serial

from donkeycar.parts.part import ThreadedPart

SHOCK = 'shock'

RAW_THROTTLE = 'raw/throttle'

RAW_STEERING = 'raw/steering'

logger = logging.getLogger(__name__)


class SerialPart(ThreadedPart):

    def __init__(self, port="/dev/ttyS0", baudrate=115200):
        self._serial = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        self._throttle_raw = None
        self._steering_raw = None
        self._shock = False
        self._on = True

    def update(self):
        logger.info("Start SerialPart")
        while self._on:
            try:
                line = self._serial.readline().decode("utf-8")
                if "shock detected" in line:
                    self._shock = True
                else:
                    fields = line.split(",")
                    self._steering_raw = fields[1]
                    self._throttle_raw = fields[2]
            except:
                logging.exception("Unexpected error")

    def run_threaded(self):
        shock = self._shock
        self._shock = False
        return self._steering_raw, self._throttle_raw, shock

    def get_inputs_keys(self) -> List[str]:
        return []

    def get_outputs_keys(self) -> List[str]:
        return [RAW_STEERING,
                RAW_THROTTLE,
                SHOCK]

    def shutdown(self):
        logger.info("Stop SerialPart")
        self._on = False
        self._serial.close()

    def run(self, **kw):
        pass
