import logging

import serial

logger = logging.getLogger(__name__)


class SerialPart:

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

    def shutdown(self):
        logger.info("Stop SerialPart")
        self._on = False
        self._serial.close()
