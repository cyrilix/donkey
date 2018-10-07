import RPi.GPIO as GPIO
from typing import List

from donkeycar.parts.arduino import DRIVE_MODE_USER
from donkeycar.parts.mqtt import USER_MODE
from donkeycar.parts.part import Part


class UserModeIndicatorLight(Part):
    def __init__(self, pin_red: int = 23, pin_green: int = 24, pin_blue: int = 25):
        self._pin_red = pin_red
        self._pin_green = pin_green
        self._pin_blue = pin_blue
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin_red, GPIO.OUT)
        GPIO.setup(pin_green, GPIO.OUT)
        GPIO.setup(pin_blue, GPIO.OUT)
        GPIO.output(pin_red, GPIO.LOW)
        GPIO.output(pin_green, GPIO.LOW)
        GPIO.output(pin_blue, GPIO.LOW)

    def run(self, user_mode: str) -> None:
        if user_mode == DRIVE_MODE_USER:
            self._set_red_value(255)
            self._set_green_value(0)
            self._set_blue_value(0)
        else:
            self._set_red_value(0)
            self._set_green_value(255)
            self._set_blue_value(0)

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
        return [USER_MODE]

    def get_outputs_keys(self) -> List[str]:
        return []
