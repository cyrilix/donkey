import logging

from donkeycar import Vehicle
from donkeycar.parts.camera_pilot import ConvertToGrayPart, \
    ContourController, AngleProcessorMiddleLine, ImagePilot, ContoursDetector, \
    ThresholdValueEstimator, ThresholdDynamicController, ThresholdStaticController, ThrottleControllerSteeringBased, \
    ThrottleControllerFixedSpeed
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

        #  Contours processing
        contours_detector = ContoursDetector(poly_dp_min=cfg.POLY_DP_MIN,
                                             arc_length_min=cfg.ARC_LENGTH_MIN,
                                             arc_length_max=cfg.ARC_LENGTH_MAX)

        threshold_value_estimator = ThresholdValueEstimator(init_value=cfg.THRESHOLD_DYNAMIC_INIT,
                                                            contours_detector=contours_detector)
        self.add(threshold_value_estimator, inputs=['img/gray'], outputs=['threshold_limit'])

        threshold_controller = self._configure_threshold(cfg)

        contours_controller = ContourController(debug=cfg.DEBUG_PILOT, contours_detector=contours_detector)
        self.add(contours_controller,
                 inputs=['img/processed'],
                 outputs=['img/contours', 'centroids'])

        # This web controller will create a web server that is capable
        # of managing steering, throttle, and modes, and more.
        custom_handlers = [
            ("/video1", VideoAPI2, {"video_part": threshold_controller}),
            ("/video2", VideoAPI2, {"video_part": contours_controller}),
            ("/video3", VideoAPI2, {"video_part": threshold_value_estimator}),
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
                                                   central_zone_in_percent=cfg.CENTRAL_ZONE_PERCENT,
                                                   use_only_first=cfg.USE_ONLY_NEAR_CONTOUR)

        throttle_controller = self._configure_throttle_controller(cfg)

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

    @staticmethod
    def _configure_throttle_controller(cfg):
        if cfg.THROTTLE_STEERING_ENABLE:
            throttle_controller = ThrottleControllerSteeringBased(min_speed=cfg.THROTTLE_MIN_SPEED,
                                                                  max_speed=cfg.THROTTLE_MAX_SPEED,
                                                                  safe_angle=cfg.THROTTLE_SAFE_ANGLE,
                                                                  dangerous_angle=cfg.THROTTLE_DANGEROUS_ANGLE)
        else:
            throttle_controller = ThrottleControllerFixedSpeed(throttle_value=cfg.THROTTLE_MAX_SPEED)
        return throttle_controller

    def _configure_car_hardware(self, cfg):
        pass

    def _configure_camera(self, cfg):
        pass

    def _configure_threshold(self, cfg):
        if cfg.THRESHOLD_DYNAMIC_ENABLE:
            logger.info("Init dynamic threshold controller")
            threshold_controller = ThresholdDynamicController(debug=cfg.DEBUG_PILOT,
                                                              threshold_default=cfg.THRESHOLD_DYNAMIC_INIT,
                                                              threshold_delta=cfg.THRESHOLD_DYNAMIC_DELTA)
            self.add(threshold_controller,
                     inputs=['img/gray', 'threshold_limit'],
                     outputs=['img/processed'])
        else:
            logger.info("Init static threshold controller")
            threshold_controller = ThresholdStaticController(debug=cfg.DEBUG_PILOT,
                                                             limit_min=cfg.THRESHOLD_LIMIT_MIN,
                                                             limit_max=cfg.THRESHOLD_LIMIT_MAX)
            self.add(threshold_controller,
                     inputs=['img/gray'],
                     outputs=['img/processed'])

        return threshold_controller
