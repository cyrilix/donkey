import cv2
import logging
from typing import List

from paho.mqtt.client import Client, MQTTMessage

from donkeycar.parts.angle import PILOT_ANGLE
from donkeycar.parts.arduino import DISTANCE_CAPTOR
from donkeycar.parts.mqtt import MqttController
from donkeycar.parts.part import Part
from donkeycar.parts.road import RoadEllipsePart, Ellipse

PILOT_THROTTLE = 'pilot/throttle'

logger = logging.getLogger(__name__)


class ThrottleConfigController(MqttController):

    def __init__(self, min_speed: float, max_speed: float, safe_angle: float, dangerous_angle: float,
                 use_steering: bool, mqtt_enable: bool = True,
                 mqtt_topic: str = 'config/throttle/#', mqtt_hostname: str = 'localhost', mqtt_port: int = 1883,
                 mqtt_client_id: str = "donkey-config-throttle-", mqtt_username: str = None, mqtt_password: str = None,
                 mqtt_qos: int = 0):
        super().__init__(mqtt_client_id, mqtt_enable, mqtt_hostname, mqtt_password, mqtt_port, mqtt_qos, mqtt_topic,
                         mqtt_username, on_message=_on_throttle_config_message)
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.safe_angle = safe_angle
        self.dangerous_angle = dangerous_angle
        self.use_steering = use_steering

    def run(self) -> (bool, float, float, float, float, bool):
        """
        :return: parts
            * cfg/throttle/compute_from_steering
            * cfg/throttle/min
            * cfg/throttle/max
            * cfg/throttle/angle/safe
            * cfg/throttle/angle/dangerous
            * cfg/throttle/stop_on_shock
        """

        return self.use_steering, self.min_speed, self.max_speed, self.safe_angle, self.dangerous_angle

    def get_inputs_keys(self) -> List[str]:
        return []

    def get_outputs_keys(self) -> List[str]:
        return ['cfg/throttle/compute_from_steering',
                'cfg/throttle/min',
                'cfg/throttle/max',
                'cfg/throttle/angle/safe',
                'cfg/throttle/angle/dangerous' ]


def _on_throttle_config_message(_: Client, userdata: ThrottleConfigController, msg: MQTTMessage):
    logger.info('new message: %s', msg.topic)
    if msg.topic.endswith("throttle/min"):
        new_value = float(msg.payload)
        logger.info("Update throttle min from %s to %s", userdata.min_speed, new_value)
        userdata.min_speed = new_value
    elif msg.topic.endswith("throttle/max"):
        new_value = float(msg.payload)
        logger.info("Update throttle max from %s to %s", userdata.max_speed, new_value)
        userdata.max_speed = new_value
    elif msg.topic.endswith("throttle/angle/safe"):
        new_value = float(msg.payload)
        logger.info("Update throttle safe angle from %s to %s", userdata.safe_angle, new_value)
        userdata.safe_angle = new_value
    elif msg.topic.endswith("throttle/angle/dangerous"):
        new_value = float(msg.payload)
        logger.info("Update throttle dangerous_angle from %s to %s", userdata.dangerous_angle, new_value)
        userdata.dangerous_angle = new_value
    elif msg.topic.endswith("throttle/stop_on_shock"):
        new_value = ("true" == msg.payload.decode('utf-8').lower())
        logger.info("Update throttle stop_on_shock from %s to %s (%s)", userdata.stop_on_shock, new_value,
                    msg.payload)
        userdata.stop_on_shock = new_value
    elif msg.topic.endswith("throttle/compute_from_steering"):
        new_value = ("true" == msg.payload.decode('utf-8').lower())
        logger.info("Update throttle compute_from_steering from %s to %s (%s)", userdata.use_steering, new_value,
                    msg.payload)
        userdata.use_steering = new_value
    else:
        logger.warning("Unexpected msg for topic %s", msg.topic)


class ThrottleControllerFixedSpeed(Part):
    def __init__(self, throttle_config_controller: ThrottleConfigController):
        self._throttle_config_controller = throttle_config_controller
        self._shock = False

    def run(self) -> float:
        return self._throttle_config_controller.min_speed

    def get_inputs_keys(self) -> List[str]:
        return []

    def get_outputs_keys(self) -> List[str]:
        return [PILOT_THROTTLE]


class ThrottleControllerSteeringBased(Part):
    """
    Implementation of throttle controller using steering value
    """

    def __init__(self, throttle_config_controller: ThrottleConfigController):
        self._throttle_config_controller = throttle_config_controller
        self._shock = False

    def run(self, angle: float) -> float:
        safe_angle = self._throttle_config_controller.safe_angle
        dangerous_angle = self._throttle_config_controller.dangerous_angle
        min_speed = self._throttle_config_controller.min_speed
        max_speed = self._throttle_config_controller.max_speed

        # Angle between 0 - safe direction ==> max_speed
        if abs(angle) < safe_angle:
            return max_speed
        # Angle > danger => min speed
        if abs(angle) > dangerous_angle:
            return min_speed

        # other ==> proportional to (max_speed - min_speed )
        speed_interv = max_speed - min_speed
        return round((abs(angle) * speed_interv) + min_speed, 2)

    def get_inputs_keys(self) -> List[str]:
        return [PILOT_ANGLE]

    def get_outputs_keys(self) -> List[str]:
        return [PILOT_THROTTLE]


class ThrottleController(Part):

    def __init__(self, throttle_config_controller: ThrottleConfigController,
                 fix_controller: ThrottleControllerFixedSpeed,
                 steering_controller: ThrottleControllerSteeringBased):
        self._throttle_config_controller = throttle_config_controller
        self.fix_controller = fix_controller
        self.steering_controller = steering_controller

    def run(self, angle: float) -> float:
        try:
            if self._throttle_config_controller.use_steering:
                return self.steering_controller.run(angle=angle)
            return self.fix_controller.run()
        except:
            logging.exception('Unexpected error')
            return self._throttle_config_controller.min_speed

    def shutdown(self):
        self.fix_controller.shutdown()
        self.steering_controller.shutdown()

    def get_inputs_keys(self) -> List[str]:
        return [PILOT_ANGLE]

    def get_outputs_keys(self) -> List[str]:
        return [PILOT_THROTTLE]


class ThrottleEllipsePart(Part):
    """
    Compute throttle from road ellipse
    """
    def __init__(self, throttle_config_controller: ThrottleConfigController):
        self._throttle_config = throttle_config_controller
        self._min_throttle = 0.4
        self._max_throttle = 0.8

    def run(self, road_ellipse: Ellipse, distance_cm: int):
        if 3 <= distance_cm <= 30:
            # Negative value in order to break
            return -0.5
        
        if not road_ellipse:
            return self._throttle_config.min_speed

        throttle_on_trust = self._compute_throttle_on_trust(road_ellipse)
        throttle_on_ellipse_ratio = self._compute_throttle_on_ellipse_ration(road_ellipse)
        throttle = throttle_on_trust * throttle_on_ellipse_ratio

        throttle = self._normalize_throttle(throttle)
        return throttle if throttle >= self._throttle_config.min_speed else self._throttle_config.min_speed

    def _compute_throttle_on_trust(self, road_ellipse: Ellipse) -> float:
        delta = self._throttle_config.max_speed - self._throttle_config.min_speed
        raw = road_ellipse.trust * delta
        throttle_on_trust = raw + self._throttle_config.min_speed
        return throttle_on_trust

    def _compute_throttle_on_ellipse_ration(self, road_ellipse: Ellipse) -> float:
        axes = road_ellipse.axes
        if not axes:
            return self._throttle_config.min_speed
        if axes[0] < axes[1]:
            return axes[0] / axes[1]
        return axes[1] / axes[0]

    def get_inputs_keys(self) -> List[str]:
        return [RoadEllipsePart.ROAD_ELLIPSE, DISTANCE_CAPTOR]

    def get_outputs_keys(self) -> List[str]:
        return [PILOT_THROTTLE]

    def _normalize_throttle(self, throttle: float) -> float:
        if throttle > self._max_throttle:
            throttle = self._max_throttle
        throttle = throttle - self._min_throttle
        if throttle < 0:
            throttle = 0
        return throttle / (self._max_throttle - self._min_throttle)


class ThrottleDebugPart(Part):
    def __init__(self, input_img_key: str):
        self._input_key = input_img_key

    def run(self, img, throttle):
        if not throttle:
            throttle = 0.0
        y_pt1 = 50
        y_pt2 = 45
        green = 255
        red = 0

        for i in range(0, int(throttle * 10)):
            cv2.rectangle(img=img,
                          pt1=(img.shape[1] - 20, y_pt1),
                          pt2=(img.shape[1] - 10, y_pt2),
                          color=(0, green, red),
                          thickness=cv2.FILLED
                        )
            y_pt1 = y_pt1 - 5
            y_pt2 = y_pt2 - 5
            red = red + 255 / 10
            green = green - 255 / 10
        img = cv2.putText(img=img, text='{0:.2f}'.format(throttle), org=(img.shape[1] - 25, 60),
                          fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=0.7, color=(255, 255, 255))
        return img

    def get_inputs_keys(self) -> List[str]:
        return [self._input_key, PILOT_THROTTLE]

    def get_outputs_keys(self) -> List[str]:
        return [self._input_key]
