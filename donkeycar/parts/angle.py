import logging
from typing import List, Tuple

import cv2
import numpy as np
from numpy.core.multiarray import ndarray
from paho.mqtt.client import Client, MQTTMessage

from donkeycar.parts.mqtt import MqttController

Centroids = List[Tuple[int, int]]

logger = logging.getLogger(__name__)


def _on_angle_config_message(client: Client, userdata, msg: MQTTMessage):
    logger.info('new message: %s', msg.topic)
    if msg.topic.endswith("angle/use_only_near_contour"):
        new_value = ("true" == msg.payload.decode('utf-8').lower())
        logger.info("Update angle use_only_near_contour from %s to %s", userdata.use_only_near_contour, new_value)
        userdata.use_only_near_contour = new_value
    elif msg.topic.endswith("angle/out_zone_percent"):
        new_value = int(msg.payload)
        logger.info("Update angle out_zone_percent from %s to %s", userdata.out_zone_percent, new_value)
        userdata.out_zone_percent = new_value
    elif msg.topic.endswith("angle/central_zone_percent"):
        new_value = int(msg.payload)
        logger.info("Update angle central_zone_percent from %s to %s", userdata.central_zone_percent, new_value)
        userdata.central_zone_percent = new_value
    else:
        logger.warning("Unexpected msg for topic %s", msg.topic)


class AngleConfigController(MqttController):

    def __init__(self, use_only_near_contour=False, out_zone_percent=20, central_zone_percent=20,
                 mqtt_enable: bool = True, mqtt_topic: str = 'config/angle/#', mqtt_hostname: str = 'localhost',
                 mqtt_port: int = 1883, mqtt_client_id: str = "donkey-config-angle-", mqtt_username: str = None,
                 mqtt_password: str = None, mqtt_qos: int = 0):
        super().__init__(mqtt_client_id, mqtt_enable, mqtt_hostname, mqtt_password, mqtt_port, mqtt_qos, mqtt_topic,
                         mqtt_username, on_message=_on_angle_config_message)
        self.use_only_near_contour = use_only_near_contour
        self.out_zone_percent = out_zone_percent
        self.central_zone_percent = central_zone_percent

    def run(self) -> (bool, int, int):
        """
        :return: parts
            * cfg/angle/use_only_near_contour
            * cfg/angle/out_zone_percent
            * cfg/angle/central_zone_percent
        """
        return self.use_only_near_contour, self.out_zone_percent, self.central_zone_percent


class AngleProcessorMiddleLine:
    """
    Angle estimation from position of middle line dots

    Compute angle from middle line position on camera

    max turn                             max turn to
      to                                    to
    right (1)                             left (2)
    <--->            <-no turn->          <--->
             (4)         (3)        (5)
    |   :            :   ||   :           :   |
    |   :            :   ||   :           :   |
    |   :            :   ||   :           :   |
                     camera axis
    <--------------- image array ------------->
    """

    def __init__(self, image_resolution=(120, 160), angle_config_controller=AngleConfigController(mqtt_enable=False)):
        self.angle_config_controller = angle_config_controller
        self._resolution = image_resolution
        self._last_value = 0

    def run(self, centroids: Centroids) -> float:
        logger.debug("Angle estimation for centroids: %s", centroids)

        if not centroids:
            logger.debug("None line found to process data")
            return self._last_value

        if len(centroids) == 1:
            angle = self._compute_angle_for_centroid(line=centroids[0][0])
        else:
            x_values = centroids[0][0]
            nb_values = 1

            if len(centroids) >= 2 and not self.angle_config_controller.use_only_near_contour:
                x_values += centroids[1][0]
                nb_values += 1

            if len(centroids) >= 4 and not self.angle_config_controller.use_only_near_contour:
                x_values += centroids[2][0]

            angle = self._compute_angle_for_centroid(line=x_values / nb_values)

        if angle < 0:
            self._last_value = -1
        if angle > 0:
            self._last_value = 1
        return angle

    def shutdown(self):
        pass

    def _compute_angle_for_centroid(self, line: float) -> float:
        # Position in percent from the left of the middle line
        pos_in_percent = line * 100 / self._resolution[1]
        logger.debug("Line position from left = %s%% (cx=%s, resolution=%s)", pos_in_percent, line, self._resolution[1])

        # convert between -1 and 1
        angle = (pos_in_percent * 2 - 100) / 100

        logger.debug("Computed angle: %s", angle)
        out_zone_delta = self.angle_config_controller.out_zone_percent * 100 / self._resolution[1] / 100
        logger.debug("Outer zone delta: %s", out_zone_delta)
        middle_zone_delta = self.angle_config_controller.central_zone_percent * 100 / self._resolution[1] / 100
        logger.debug("Middle zone delta: %s", out_zone_delta)

        logger.debug("Angle fixed: %s", str(angle))
        if angle < -1.0 + out_zone_delta:
            # zone (1)
            angle = -1.0
        elif 0 > angle > - middle_zone_delta:
            # zone (3) left
            angle = 0.0
        elif 0 < angle < middle_zone_delta:
            # zone (3) right
            angle = 0.0
        elif angle > 1.0 - out_zone_delta:
            # zone (2)
            angle = 1.0
        logger.debug("Angle fixed: %s", str(angle))
        return angle


class AngleDebug:

    def __init__(self, config: AngleConfigController):
        self._config = config

    def shutdown(self):
        pass

    def run(self, img: ndarray) -> ndarray:
        try:
            rows, columns, channel = np.shape(img)
            middle = int(columns / 2)
            mask = np.zeros(img.shape, np.uint8)
            central_zone_delta = int(((columns / 100) * self._config.central_zone_percent) / 2)

            # Draw safe zone
            cv2.rectangle(img=mask,
                          pt1=(middle - central_zone_delta, 0),
                          pt2=(middle + central_zone_delta, rows),
                          color=(0, 255, 0),
                          thickness=cv2.FILLED)

            out_zone_delta = int(((columns / 100) * self._config.out_zone_percent) / 2)

            # Draw dangerous zone
            cv2.rectangle(img=mask,
                          pt1=(0, 0),
                          pt2=(out_zone_delta, rows),
                          color=(255, 0, 0),
                          thickness=cv2.FILLED)
            cv2.rectangle(img=mask,
                          pt1=(columns - out_zone_delta, 0),
                          pt2=(columns, rows),
                          color=(255, 0, 0),
                          thickness=cv2.FILLED)

            # Apply mask
            img_debug = cv2.addWeighted(src1=img.copy(), alpha=0.7,
                                        src2=mask, beta=0.3,
                                        gamma=0)

            # Draw central axes
            img_debug = cv2.line(img=img_debug,
                                 pt1=(middle, 0),
                                 pt2=(middle, img.shape[1]),
                                 color=(0, 0, 255),
                                 thickness=2)
            return img_debug
        except:
            logging.exception("Unexpected error")
            return np.zeros(img.shape)
