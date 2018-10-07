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
# import parts
from donkeycar.parts.camera import ImageListCamera
from donkeycar.templates.fousduvolant_base import BaseVehicle

logger = logging.getLogger(__name__)


class MalabilleCar(BaseVehicle):
    def _configure_car_hardware(self, cfg):
        pass

    def _configure_camera(self, cfg):
        self.register(ImageListCamera(path_mask='~/src/robocars/d2rd/data/tub_1_18-02-10/*.jpg'))

    def _configure_arduino(self, cfg):
        pass


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    args = docopt(__doc__)
    cfg = dk.load_config()

    if args['drive']:
        vehicle = MalabilleCar(cfg)

        # run the vehicle
        vehicle.start(rate_hz=cfg.DRIVE_LOOP_HZ,
                      max_loop_count=cfg.MAX_LOOPS)
