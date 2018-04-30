import logging
from typing import List

from paho.mqtt.client import Client, MQTTMessage

from donkeycar.parts.mqtt import MqttController

logger = logging.getLogger(__name__)


def _on_throttle_config_message(client: Client, userdata, msg: MQTTMessage):
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


class ThrottleConfigController(MqttController):

    def __init__(self,
                 min_speed: float, max_speed: float, safe_angle: float, dangerous_angle: float, use_steering: bool,
                 stop_on_shock: bool,
                 mqtt_enable: bool = True, mqtt_topic: str = 'config/throttle/#',
                 mqtt_hostname: str = 'localhost', mqtt_port: int = 1883,
                 mqtt_client_id: str = "donkey-config-throttle-", mqtt_username: str = None, mqtt_password: str = None,
                 mqtt_qos: int = 0,
                 ):
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.safe_angle = safe_angle
        self.dangerous_angle = dangerous_angle
        self.use_steering = use_steering
        self.stop_on_shock = stop_on_shock
        self._init_mqtt(mqtt_client_id, mqtt_enable, mqtt_hostname, mqtt_password, mqtt_port, mqtt_qos, mqtt_topic,
                        mqtt_username, on_message=_on_throttle_config_message)

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

        return self.use_steering, self.min_speed, self.max_speed, self.safe_angle, self.dangerous_angle, \
               self.stop_on_shock

    @staticmethod
    def get_outputs_name_parts() -> List[str]:
        return ['cfg/throttle/compute_from_steering',
                'cfg/throttle/min',
                'cfg/throttle/max',
                'cfg/throttle/angle/safe',
                'cfg/throttle/angle/dangerous',
                'cfg/throttle/stop_on_shock']


class ThrottleControllerFixedSpeed:

    def __init__(self):
        self._shock = False

    def shutdown(self):
        pass

    def run(self, throttle_value: float, stop_on_shock: bool = False, shock: bool = False) -> float:
        if not stop_on_shock:
            return throttle_value

        if shock:
            logger.info("!!!!!!! SHOCK DETECTED !!!!!!!!")
            self._shock = shock

        if self._shock:
            return 0.0
        else:
            return throttle_value


class ThrottleControllerSteeringBased:
    """
    Implementation of throttle controller using steering value
    """

    def __init__(self):
        self._shock = False

    def shutdown(self):
        pass

    def run(self, angle: float, min_speed: float, max_speed: float, safe_angle: float, dangerous_angle: float,
            stop_on_shock: bool, shock: bool) -> float:
        if stop_on_shock:
            if shock:
                logger.info("!!!!!!! SHOCK DETECTED !!!!!!!!")
                self._shock = shock

            if self._shock:
                return 0.0

        # Angle between 0 - safe direction ==> max_speed
        if angle < safe_angle:
            return max_speed
        # Angle > danger => min speed
        if angle > dangerous_angle:
            return min_speed

        # other ==> proportional to (max_speed - min_speed )
        speed_interv = max_speed - min_speed
        return round((angle * speed_interv) + min_speed, 2)


class ThrottleController:

    def __init__(self, fix_controller: ThrottleControllerFixedSpeed,
                 steering_controller: ThrottleControllerSteeringBased):
        self.fix_controller = fix_controller
        self.steering_controller = steering_controller

    def run(self, angle: float, use_steering: bool, throttle_min: float, throttle_max: float, angle_saf: float,
            angle_dangerous: float, stop_on_shock: bool = False, shock: bool = False) -> float:
        try:
            if use_steering:
                return self.steering_controller.run(angle=angle, min_speed=throttle_min, max_speed=throttle_max,
                                                    safe_angle=angle_saf, dangerous_angle=angle_dangerous,
                                                    stop_on_shock=stop_on_shock, shock=shock)
            return self.fix_controller.run(throttle_value=throttle_min, stop_on_shock=stop_on_shock, shock=shock)
        except:
            logging.exception('Unexpected error')
            return throttle_min

    def shutdown(self):
        self.fix_controller.shutdown()
        self.steering_controller.shutdown()
