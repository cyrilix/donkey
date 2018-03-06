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
from donkeycar.parts.camera import WebcamCV
from donkeycar.parts.camera_pilot import ImagePilot, AngleProcessorMiddleLine, ThrottleControllerFixedSpeed, \
    ThresholdController, ContourController
from donkeycar.parts.controller import LocalWebController
from donkeycar.parts.datastore import TubHandler
from donkeycar.parts.transform import Lambda
from donkeycar.parts.web_controller.web import VideoAPI2

logger = logging.getLogger(__name__)


def drive(cfg):
    """
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag `threaded`.

    All parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
    Parts may have named outputs and inputs. The framework handles passing named outputs
    to parts requesting the same named input.
    """

    # Initialize car
    V = dk.vehicle.Vehicle()
    cam = WebcamCV(resolution=cfg.CAMERA_RESOLUTION)
    V.add(cam, outputs=['cam/image_array'], threaded=True)

    # Cleaning image before processing
    threshold_controller = ThresholdController(
        threshold_limit=cfg.THRESHOLD_LIMIT,
        debug=cfg.DEBUG_PILOT)
    V.add(threshold_controller,
          inputs=['cam/image_array'],
          outputs=['img/processed'])

    #  Contours processing
    contours_controller = ContourController(debug=cfg.DEBUG_PILOT)
    V.add(contours_controller,
          inputs=['img/processed'],
          outputs=['img/contours', 'centroids'])

    # This web controller will create a web server that is capable
    # of managing steering, throttle, and modes, and more.
    custom_handlers = [
        ("/video1", VideoAPI2, {"video_part": threshold_controller}),
        ("/video2", VideoAPI2, {"video_part": contours_controller}),
    ]
    ctr = LocalWebController(custom_handlers=custom_handlers)
    V.add(ctr,
          inputs=['cam/image_array'],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
          threaded=True)

    # See if we should even run the pilot module.
    # This is only needed because the part run_condition only accepts boolean
    def pilot_condition(mode):
        if mode == 'user':
            return False
        else:
            return True

    pilot_condition_part = Lambda(pilot_condition)
    V.add(pilot_condition_part, inputs=['user/mode'], outputs=['run_pilot'])

    # Run the pilot if the mode is not user.
    angle_processor = AngleProcessorMiddleLine(image_resolution=cfg.CAMERA_RESOLUTION,
                                               out_zone_in_percent=cfg.OUT_ZONE_PERCENT,
                                               central_zone_in_percent=cfg.CENTRAL_ZONE_PERCENT)
    throttle_controller = ThrottleControllerFixedSpeed(throttle_value=cfg.THROTTLE_MAX_SPEED)
    camera_pilot = ImagePilot(angle_estimator=angle_processor,
                              throttle_controller=throttle_controller)

    V.add(camera_pilot, inputs=['centroids'],
          outputs=['pilot/angle', 'pilot/throttle'],
          run_condition='run_pilot')

    # Choose what inputs should change the car.
    def drive_mode(mode,
                   user_angle, user_throttle,
                   pilot_angle, pilot_throttle):
        if mode == 'user':
            return user_angle, user_throttle

        elif mode == 'local_angle':
            return pilot_angle, user_throttle

        else:
            return pilot_angle, pilot_throttle

    drive_mode_part = Lambda(drive_mode)
    V.add(drive_mode_part,
          inputs=['user/mode', 'user/angle', 'user/throttle',
                  'pilot/angle', 'pilot/throttle'],
          outputs=['angle', 'throttle'])

    gpio_motor = GpioMotor()
    V.add(gpio_motor, inputs=['throttle', 'angle'])

    # add tub to save data
    inputs = ['cam/image_array', 'user/angle', 'user/throttle', 'user/mode']
    types = ['image_array', 'float', 'float', 'str']

    th = TubHandler(path=cfg.DATA_PATH)
    tub = th.new_tub_writer(inputs=inputs, types=types)
    V.add(tub, inputs=inputs, run_condition='recording')

    # run the vehicle
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ,
            max_loop_count=cfg.MAX_LOOPS)

    logger.info("You can now go to <your pi ip address>:8887 to drive your car.")


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    args = docopt(__doc__)
    cfg = dk.load_config()

    if args['drive']:
        drive(cfg)
