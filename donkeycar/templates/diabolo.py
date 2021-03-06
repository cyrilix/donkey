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
from donkeycar.parts.actuator import GpioMotor
# import parts
from donkeycar.parts.camera import PiCamera
from donkeycar.templates.fousduvolant_base import BaseVehicle

logger = logging.getLogger(__name__)


class Diabolo(BaseVehicle):

    def _configure_car_hardware(self, cfg):
        self.register(GpioMotor())

    def _configure_camera(self, cfg):
        self.register(PiCamera(resolution=cfg.CAMERA_RESOLUTION, rotation=180))

    def _configure_arduino(self, cfg):
        pass


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    args = docopt(__doc__)
    cfg = dk.load_config()

    if args['drive']:
        vehicle = Diabolo(cfg)

        # run the vehicle
        vehicle.start(rate_hz=cfg.DRIVE_LOOP_HZ,
                      max_loop_count=cfg.MAX_LOOPS)
