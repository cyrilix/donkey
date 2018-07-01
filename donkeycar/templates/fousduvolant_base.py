import logging
import platform

from donkeycar import Vehicle
from donkeycar.parts.actuator import ANGLE, THROTTLE
from donkeycar.parts.angle import AngleProcessorMiddleLine, AngleConfigController, AngleDebug, PILOT_ANGLE, \
    AngleContourDebug
from donkeycar.parts.arduino import SerialPart, DRIVE_MODE_USER, DRIVE_MODE_LOCAL_ANGLE
from donkeycar.parts.img_process import ConvertToGrayPart, HistogramPart, GraySelectorPart
from donkeycar.parts.mqtt import MqttDrive, USER_MODE
from donkeycar.parts.mqtt import MultiProcessingMetringPublisher
from donkeycar.parts.road import RoadPart, RoadDebugPart, RoadConfigController
from donkeycar.parts.threshold import ThresholdConfigController, ThresholdController, ThresholdValueEstimator, \
    ContoursDetector, ContourController, ContoursConfigController, ThresholdValueEstimatorConfig
from donkeycar.parts.throttle import ThrottleControllerSteeringBased, ThrottleControllerFixedSpeed, \
    ThrottleController, ThrottleConfigController, PILOT_THROTTLE
from donkeycar.parts.transform import Lambda
from donkeycar.parts.web_controller.web import LocalWebController, USER_ANGLE, USER_THROTTLE

logger = logging.getLogger(__name__)


class BaseVehicle(Vehicle):

    def __init__(self, cfg):
        if not cfg.MQTT_ENABLE:
            mqtt_publisher = None
        else:
            mqtt_publisher = MultiProcessingMetringPublisher(topic='fousduvolant/' + platform.node(),
                                                             client_id=platform.node(),
                                                             mqtt_address=(cfg.MQTT_HOSTNAME, cfg.MQTT_PORT),
                                                             qos=cfg.MQTT_QOS,
                                                             mqtt_user=platform.node(),
                                                             mqtt_password=platform.node())
        super().__init__(metrics_publisher=mqtt_publisher)
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
        self._configure_arduino(cfg)

        # Convert image to gray
        self.register(ConvertToGrayPart())
        self.register(HistogramPart())
        self.register(GraySelectorPart())

        contours_detector = self._configure_contours_detector(cfg)

        self._configure_threshold_value_estimator(cfg, contour_detector=contours_detector)

        self._configure_threshold(cfg)

        self.register(ContourController(contours_detector=contours_detector))

        self._configure_road_detection()

        # This web controller will create a web server that is capable
        # of managing steering, throttle, and modes, and more.
        self.register(LocalWebController())
        self.register(MqttDrive())

        self._configure_angle_part(cfg)

        self._configure_throttle_controller(cfg)

        # Choose what inputs should change the car.
        def drive_mode(mode,
                       user_angle, user_throttle,
                       pilot_angle, pilot_throttle):
            if mode == DRIVE_MODE_USER:
                return user_angle, user_throttle

            elif mode == DRIVE_MODE_LOCAL_ANGLE:
                return pilot_angle, user_throttle

            else:
                return pilot_angle, pilot_throttle

        drive_mode_part = Lambda(drive_mode,
                                 inputs=[USER_MODE, USER_ANGLE, USER_THROTTLE, PILOT_ANGLE, PILOT_THROTTLE],
                                 outputs=[ANGLE, THROTTLE])

        self.register(drive_mode_part)
        self._configure_car_hardware(cfg)
        self._configure_indicators(cfg)

        logger.info("You can now go to <your pi ip address>:8887 to drive your car.")

    def _configure_threshold_value_estimator(self, cfg, contour_detector: ContoursDetector):
        threshold_value_estimator_config = ThresholdValueEstimatorConfig(centroid_value=cfg.THRESHOLD_DYNAMIC_INIT)
        self.register(threshold_value_estimator_config)
        threshold_value_estimator = ThresholdValueEstimator(config=threshold_value_estimator_config,
                                                            contours_detector=contour_detector)
        self.register(threshold_value_estimator)

    def _configure_contours_detector(self, cfg):
        #  Contours processing
        config = ContoursConfigController(poly_dp_min=cfg.POLY_DP_MIN,
                                          poly_dp_max=cfg.POLY_DP_MAX,
                                          arc_length_min=cfg.ARC_LENGTH_MIN,
                                          arc_length_max=cfg.ARC_LENGTH_MAX,
                                          mqtt_enable=True)
        contours_detector = ContoursDetector(config=config)
        self.register(config)
        return contours_detector

    def _configure_angle_part(self, cfg):
        config = AngleConfigController(number_centroids_to_use=cfg.NB_CONTOURS_TO_USE,
                                       out_zone_percent=cfg.OUT_ZONE_PERCENT,
                                       central_zone_percent=cfg.CENTRAL_ZONE_PERCENT)
        self.register(config)
        self.register(AngleProcessorMiddleLine(image_resolution=cfg.CAMERA_RESOLUTION,
                                               angle_config_controller=config))
        self.register(AngleDebug(config=config))
        self.register(AngleContourDebug(config=config))

    def _configure_throttle_controller(self, cfg):
        config_controller = ThrottleConfigController(stop_on_shock=cfg.THROTTLE_STOP_ON_SHOCK,
                                                     min_speed=cfg.THROTTLE_MIN_SPEED,
                                                     max_speed=cfg.THROTTLE_MAX_SPEED,
                                                     safe_angle=cfg.THROTTLE_SAFE_ANGLE,
                                                     dangerous_angle=cfg.THROTTLE_DANGEROUS_ANGLE,
                                                     use_steering=cfg.THROTTLE_STEERING_ENABLE)
        self.register(config_controller)
        self.register(ThrottleController(throttle_config_controller=config_controller,
                                         fix_controller=ThrottleControllerFixedSpeed(
                                             throttle_config_controller=config_controller),
                                         steering_controller=ThrottleControllerSteeringBased(
                                             throttle_config_controller=config_controller)))

    def _configure_arduino(self, cfg):
        self.register(part=SerialPart(port=cfg.ARDUINO_SERIAL_PORT, baudrate=cfg.ARDUINO_SERIAL_BAUDRATE))

    def _configure_car_hardware(self, cfg):
        pass

    def _configure_camera(self, cfg):
        pass

    def _configure_indicators(self, cfg):
        pass

    def _configure_threshold(self, cfg):
        logger.info("Init threshold controller")
        limit_min = cfg.THRESHOLD_LIMIT_MIN
        limit_max = cfg.THRESHOLD_LIMIT_MAX
        dynamic_enabled = cfg.THRESHOLD_DYNAMIC_ENABLE
        dynamic_default_threshold = cfg.THRESHOLD_DYNAMIC_INIT
        dynamic_delta = cfg.THRESHOLD_DYNAMIC_DELTA
        horizon = cfg.THRESHOLD_HORIZON
        threshold_config = ThresholdConfigController(limit_min=limit_min, limit_max=limit_max,
                                                     threshold_dynamic=dynamic_enabled,
                                                     threshold_default=dynamic_default_threshold,
                                                     threshold_delta=dynamic_delta, horizon=horizon)
        self.register(threshold_config)
        self.register(ThresholdController(config=threshold_config))

    def _configure_road_detection(self):
        config = RoadConfigController(mqtt_enable=True)
        self.register(config)
        self.register(RoadPart(config=config))
        self.register(RoadDebugPart())
