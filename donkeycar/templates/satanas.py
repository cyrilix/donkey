#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car and train a model for it.

Usage:
    manage.py (drive) [--model=<model>] [--js]

Options:
    -h --help        Show this screen.
"""
import logging

from docopt import docopt

import donkeycar as dk
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle
from donkeycar.parts.arduino import SerialPart
from donkeycar.parts.camera import PiCamera
from donkeycar.parts.indicators import UserModeIndicatorLight
from donkeycar.templates.fousduvolant_base import BaseVehicle

logger = logging.getLogger(__name__)


class Satanas(BaseVehicle):

    def _configure_car_hardware(self, cfg):
        steering_controller = PCA9685(cfg.STEERING_CHANNEL)
        steering = PWMSteering(controller=steering_controller,
                               left_pulse=cfg.STEERING_LEFT_PWM,
                               right_pulse=cfg.STEERING_RIGHT_PWM)

        throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL)
        throttle = PWMThrottle(controller=throttle_controller,
                               max_pulse=cfg.THROTTLE_FORWARD_PWM,
                               zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                               min_pulse=cfg.THROTTLE_REVERSE_PWM)

        self.register(steering)
        self.register(throttle)
        self.register(SerialPart(port=cfg.ARDUINO_SERIAL_PORT, baudrate=cfg.ARDUINO_SERIAL_BAUDRATE))

    def _configure_camera(self, cfg):
        self.register(PiCamera(resolution=cfg.CAMERA_RESOLUTION))

    def _configure_indicators(self, cfg):
        self.register(UserModeIndicatorLight(pin_red=23, pin_green=24, pin_blue=25))


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    args = docopt(__doc__)
    cfg = dk.load_config()

    if args['drive']:
        vehicle = Satanas(cfg)

        # run the vehicle
        vehicle.start(rate_hz=cfg.DRIVE_LOOP_HZ,
                      max_loop_count=cfg.MAX_LOOPS)
