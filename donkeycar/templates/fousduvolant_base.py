import logging

from donkeycar import Vehicle
from donkeycar.parts.camera_pilot import ConvertToGrayPart, ThresholdController, \
    ContourController, AngleProcessorMiddleLine, ThrottleControllerFixedSpeed, ImagePilot
from donkeycar.parts.datastore import TubHandler
from donkeycar.parts.transform import Lambda
from donkeycar.parts.web_controller.web import VideoAPI2, LocalWebController

logger = logging.getLogger(__name__)


class BaseVehicle(Vehicle):

    def __init__(self, cfg):
        super().__init__()
        self._configure(cfg)

    def _configure(self, cfg):
        """
        Construct a working robotic vehicle from many parts.
        Each part runs as a job in the Vehicle loop, calling either
        it's run or run_threaded method depending on the constructor flag `threaded`.

        All parts are updated one after another at the framerate given in
        cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
        Parts may have named outputs and inputs. The framework handles passing named outputs
        to parts requesting the same named input.
        """

        self._configure_camera(cfg)

        # Convert image to gray
        self.add(ConvertToGrayPart(), inputs=['cam/image_array'], outputs=['img/gray'])
        # threshold_value_estimator = ThresholdValueEstimator(init_value=cfg.THRESHOLD_LIMIT)
        # self.add(threshold_value_estimator, inputs=['img/gray'], outputs=['threshold_limit'])

        # Cleaning image before processing
        threshold_controller = ThresholdController(debug=cfg.DEBUG_PILOT,
                                                   limit_min=cfg.THRESHOLD_LIMIT_MIN,
                                                   limit_max=cfg.THRESHOLD_LIMIT_MAX)
        self.add(threshold_controller,
                 inputs=['img/gray'],
                 outputs=['img/processed'])

        #  Contours processing
        contours_controller = ContourController(debug=cfg.DEBUG_PILOT)
        self.add(contours_controller,
                 inputs=['img/processed'],
                 outputs=['img/contours', 'centroids'])

        # Warn: cyclic dependencies
        #self.add(threshold_value_estimator,
        #         inputs=['img/gray'],
        #         outputs=['threshold'],
        #         threaded=True)

        # This web controller will create a web server that is capable
        # of managing steering, throttle, and modes, and more.
        custom_handlers = [
            ("/video1", VideoAPI2, {"video_part": threshold_controller}),
            ("/video2", VideoAPI2, {"video_part": contours_controller}),
         #   ("/video3", VideoAPI2, {"video_part": threshold_value_estimator}),
        ]
        ctr = LocalWebController(custom_handlers=custom_handlers)
        self.add(ctr,
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
        self.add(pilot_condition_part, inputs=['user/mode'], outputs=['run_pilot'])

        # Run the pilot if the mode is not user.
        angle_processor = AngleProcessorMiddleLine(image_resolution=cfg.CAMERA_RESOLUTION,
                                                   out_zone_in_percent=cfg.OUT_ZONE_PERCENT,
                                                   central_zone_in_percent=cfg.CENTRAL_ZONE_PERCENT)
        throttle_controller = ThrottleControllerFixedSpeed(throttle_value=cfg.THROTTLE_MAX_SPEED)
        camera_pilot = ImagePilot(angle_estimator=angle_processor,
                                  throttle_controller=throttle_controller)

        self.add(camera_pilot,
                 inputs=['centroids'],
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
        self.add(drive_mode_part,
                 inputs=['user/mode', 'user/angle', 'user/throttle',
                         'pilot/angle', 'pilot/throttle'],
                 outputs=['angle', 'throttle'])

        self._configure_car_hardware(cfg)

        # add tub to save data
        inputs = ['cam/image_array', 'user/angle', 'user/throttle', 'user/mode']
        types = ['image_array', 'float', 'float', 'str']

        th = TubHandler(path=cfg.DATA_PATH)
        tub = th.new_tub_writer(inputs=inputs, types=types)
        self.add(tub, inputs=inputs, run_condition='recording')

        logger.info("You can now go to <your pi ip address>:8887 to drive your car.")

    def _configure_car_hardware(self, cfg):
        pass

    def _configure_camera(self, cfg):
        pass
