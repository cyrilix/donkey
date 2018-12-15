import logging
import platform

from donkeycar import Vehicle
from donkeycar.parts.actuator import ANGLE, THROTTLE
from donkeycar.parts.angle import AngleProcessorMiddleLine, AngleConfigController, AngleDebug, PILOT_ANGLE, \
    AngleContourDebug, AngleRoadPart
from donkeycar.parts.arduino import SerialPart, DRIVE_MODE_USER, DRIVE_MODE_LOCAL_ANGLE, USER_THROTTLE, USER_ANGLE
from donkeycar.parts.mqtt import MultiProcessingMetringPublisher
from donkeycar.parts.mqtt import USER_MODE
from donkeycar.parts.road import ComponentRoadPart
from donkeycar.parts.throttle import ThrottleControllerSteeringBased, ThrottleControllerFixedSpeed, \
    ThrottleController, ThrottleConfigController, PILOT_THROTTLE
from donkeycar.parts.transform import Lambda
from donkeycar.parts.web_controller.web import LocalWebController

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
        self.register(ComponentRoadPart())

        # This web controller will create a web server that is capable
        # of managing steering, throttle, and modes, and more.
        self.register(LocalWebController())
        self._configure_arduino(cfg)

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

    def _configure_arduino(self, cfg):
        self.register(part=SerialPart(port=cfg.ARDUINO_SERIAL_PORT, baudrate=cfg.ARDUINO_SERIAL_BAUDRATE))

    def _configure_car_hardware(self, cfg):
        pass

    def _configure_camera(self, cfg):
        pass

    def _configure_indicators(self, cfg):
        pass

    def _configure_throttle_controller(self, cfg):
        self.register(ThrottleControllerFixedSpeed(speed=cfg.THROTTLE_MIN_SPEED))

    def _configure_angle_part(self, cfg):
        config = AngleConfigController(number_centroids_to_use=cfg.NB_CONTOURS_TO_USE,
                                       out_zone_percent=cfg.OUT_ZONE_PERCENT,
                                       central_zone_percent=cfg.CENTRAL_ZONE_PERCENT)
        self.register(config)
        self.register(AngleRoadPart(image_resolution=cfg.CAMERA_RESOLUTION, angle_config_controller=config))
        # self.register(AngleDebug(config=config))
