import time
from typing import List

import RPi.GPIO as GPIO

from donkeycar.parts.arduino import DRIVE_MODE_USER, DRIVE_MODE_PILOT
from donkeycar.parts.mqtt import USER_MODE, CTRL_RECORD
from donkeycar.parts.part import Part, ThreadedPart
from donkeycar.parts.road import RoadEllipsePart, Ellipse


class UserModeIndicatorLight(ThreadedPart):

    def __init__(self, pin_red: int = 23, pin_green: int = 24, pin_blue: int = 25,
                 blink_delta_sec: float = 1.0):
        self._blink_delta = blink_delta_sec
        self._pin_red = pin_red
        self._pin_green = pin_green
        self._pin_blue = pin_blue
        self._blink = False
        self._next_blink = time.time() + self._blink_delta
        self._blink_on = True
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin_red, GPIO.OUT)
        GPIO.setup(pin_green, GPIO.OUT)
        GPIO.setup(pin_blue, GPIO.OUT)
        GPIO.output(pin_red, GPIO.LOW)
        GPIO.output(pin_green, GPIO.LOW)
        GPIO.output(pin_blue, GPIO.LOW)

    def update(self):
        pass

    def run_threaded(self, user_mode: str, road_ellipse: Ellipse, ctrl_record: bool) -> None:
        if ctrl_record and time.time() > self._next_blink:
            self._next_blink = time.time() + self._blink_delta
        if self._blink_on and ctrl_record:
            self._set_red_value(0)
            self._set_green_value(0)
            self._set_blue_value(0)
            self._blink_on = False
            return

        if user_mode == DRIVE_MODE_USER:
            self._set_red_value(0)
            self._set_green_value(255)
            self._set_blue_value(0)
        elif user_mode == DRIVE_MODE_PILOT:
            if not road_ellipse or not road_ellipse.trust:
                red = 255
                blue = 0
            else:
                red = int(255 * (1 - road_ellipse.trust))
                blue = int(255 * road_ellipse.trust)
            self._set_red_value(red)
            self._set_green_value(0)
            self._set_blue_value(blue)
        else:
            self._set_red_value(0)
            self._set_green_value(0)
            self._set_blue_value(255)

    def _set_red_value(self, value: int):
        if value == 0:
            GPIO.output(self._pin_red, GPIO.LOW)
        else:
            GPIO.output(self._pin_red, GPIO.HIGH)

    def _set_green_value(self, value: int):
        if value == 0:
            GPIO.output(self._pin_green, GPIO.LOW)
        else:
            GPIO.output(self._pin_green, GPIO.HIGH)

    def _set_blue_value(self, value: int):
        if value == 0:
            GPIO.output(self._pin_blue, GPIO.LOW)
        else:
            GPIO.output(self._pin_blue, GPIO.HIGH)

    def shutdown(self):
        self._set_red_value(0)
        self._set_green_value(0)
        self._set_blue_value(0)
        GPIO.cleanup()

    def get_inputs_keys(self) -> List[str]:
        return [USER_MODE, RoadEllipsePart.ROAD_ELLIPSE, CTRL_RECORD]

    def get_outputs_keys(self) -> List[str]:
        return []
