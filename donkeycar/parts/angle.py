import logging
from collections import namedtuple
from typing import List, Tuple, Optional

import cv2
import numpy as np
from numpy.core.multiarray import ndarray
from paho.mqtt.client import Client, MQTTMessage

from donkeycar.parts.camera import CAM_IMAGE
from donkeycar.parts.mqtt import MqttController
from donkeycar.parts.part import Part
from donkeycar.parts.road import RoadPart, RoadDebugPart
from donkeycar.parts.threshold import CONTOURS_CENTROIDS, Shape, CONTOURS_SHAPES, IMG_CONTOURS

IMG_ANGLE_ZONE = 'img/angle_zone'
IMG_ANGLE_CONTOURS = 'img/angle_contours'

CFG_ANGLE_CENTRAL_ZONE_PERCENT = 'cfg/angle/central_zone_percent'

CFG_ANGLE_OUT_ZONE_PERCENT = 'cfg/angle/out_zone_percent'

CFG_ANGLE_NUMBER_CENTROIDS_TO_USE = 'cfg/angle/number_centroids_to_use'

PILOT_ANGLE = 'pilot/angle'

Centroid = Tuple[int, int]

logger = logging.getLogger(__name__)


class AngleConfigController(MqttController):

    def __init__(self, number_centroids_to_use=1, out_zone_percent=20, central_zone_percent=20,
                 mqtt_enable: bool = True, mqtt_topic: str = 'config/angle/#', mqtt_hostname: str = 'localhost',
                 mqtt_port: int = 1883, mqtt_client_id: str = "donkey-config-angle-", mqtt_username: str = None,
                 mqtt_password: str = None, mqtt_qos: int = 0):
        super().__init__(mqtt_client_id, mqtt_enable, mqtt_hostname, mqtt_password, mqtt_port, mqtt_qos, mqtt_topic,
                         mqtt_username, on_message=_on_angle_config_message)
        self.number_centroids_to_use = number_centroids_to_use
        self.out_zone_percent = out_zone_percent
        self.central_zone_percent = central_zone_percent

    def run(self) -> (int, int, int):
        """
        :return: parts
            * cfg/angle/number_centroids_to_use
            * cfg/angle/out_zone_percent
            * cfg/angle/central_zone_percent
        """
        return self.number_centroids_to_use, self.out_zone_percent, self.central_zone_percent

    def get_inputs_keys(self) -> List[str]:
        return []

    def get_outputs_keys(self) -> List[str]:
        return [CFG_ANGLE_NUMBER_CENTROIDS_TO_USE,
                CFG_ANGLE_OUT_ZONE_PERCENT,
                CFG_ANGLE_CENTRAL_ZONE_PERCENT]


def _on_angle_config_message(_: Client, userdata: AngleConfigController, msg: MQTTMessage):
    logger.info('new message: %s', msg.topic)
    if msg.topic.endswith("angle/number_centroids_to_use"):
        new_value = int(msg.payload)
        logger.info("Update angle number centroids to use from %s to %s", userdata.number_centroids_to_use, new_value)
        userdata.number_centroids_to_use = new_value
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


class CentroidToAngleProcessor:

    def __init__(self, img_resolution: Tuple[int, int],
                 angle_config_controller: AngleConfigController):
        self._angle_config_controller = angle_config_controller
        self._resolution = img_resolution

    def compute_angle_for_centroid(self, line: float) -> float:
        # Position in percent from the left of the middle line
        pos_in_percent = line * 100 / self._resolution[1]
        logger.debug("Line position from left = %s%% (cx=%s, resolution=%s)", pos_in_percent, line, self._resolution[1])

        # convert between -1 and 1
        angle = (pos_in_percent * 2 - 100) / 100

        logger.debug("Computed angle: %s", angle)
        out_zone_delta = self._angle_config_controller.out_zone_percent * 100 / self._resolution[1] / 100
        logger.debug("Outer zone delta: %s", out_zone_delta)
        middle_zone_delta = self._angle_config_controller.central_zone_percent * 100 / self._resolution[1] / 100
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


class AngleProcessorMiddleLine(Part):
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

    def run(self, centroids: List[Centroid]) -> float:
        logger.debug("Angle estimation for centroids: %s", centroids)

        if not centroids:
            logger.debug("None line found to process data")
            return self._last_value

        nb_centroids = len(centroids)
        if nb_centroids > self.angle_config_controller.number_centroids_to_use:
            nb_centroids = self.angle_config_controller.number_centroids_to_use

        x_values = [x[0] for x in centroids[0:nb_centroids]]
        weighted_mean_x = np.average(x_values, weights=range(nb_centroids + 1, 1, -1))
        angle = self._compute_angle_for_centroid(line=weighted_mean_x)

        if angle < 0:
            self._last_value = -1
        if angle > 0:
            self._last_value = 1
        return angle

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

    def get_inputs_keys(self) -> List[str]:
        return [CONTOURS_CENTROIDS]

    def get_outputs_keys(self) -> List[str]:
        return [PILOT_ANGLE]


class AngleDebug(Part):

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

    def get_inputs_keys(self) -> List[str]:
        return [CAM_IMAGE]

    def get_outputs_keys(self) -> List[str]:
        return [IMG_ANGLE_ZONE]


class AngleContourDebug(Part):

    def __init__(self, config: AngleConfigController):
        self._config = config

    def shutdown(self):
        pass

    def run(self, img: ndarray, shapes: List[Shape]) -> ndarray:
        try:
            img_debug = img.copy()
            nb_contours = self._config.number_centroids_to_use
            colors = self._get_colors_index(nb_contours)
            for i in range(nb_contours):
                cv2.drawContours(img_debug, shapes[i:i + 1], -1, colors[i], 2)
            return img_debug
        except:
            logging.exception("Unexpected error")
            return np.zeros(img.shape)

    @staticmethod
    def _get_colors_index(palette_size: int) -> List[Tuple[int, int, int]]:
        max_blue = 250
        min_blue = 150
        delta_blue = int((max_blue - min_blue) / palette_size)

        max_red = 100
        min_red = 0
        delta_red = int((max_red - min_red) / palette_size)

        max_green = 150
        min_green = 50
        delta_green = int((max_green - min_green) / palette_size)

        colors = []
        red = min_red
        green = max_green
        blue = max_blue

        for _ in range(palette_size):
            colors.append((red, green, blue))
            red = red + delta_red
            green = green - delta_green
            blue = blue - delta_blue
        return colors

    def get_inputs_keys(self) -> List[str]:
        return [IMG_CONTOURS, CONTOURS_SHAPES]

    def get_outputs_keys(self) -> List[str]:
        return [IMG_ANGLE_CONTOURS]


Ellipse = namedtuple('Ellipse', ('center', 'axes', 'angle'))


class AngleRoadPart(Part):
    ROAD_ELLIPSE = 'road/ellipse'

    def __init__(self, image_resolution=(120, 160), angle_config_controller=AngleConfigController(mqtt_enable=False)):
        self.angle_processor = CentroidToAngleProcessor(img_resolution=image_resolution,
                                                        angle_config_controller=angle_config_controller)

    def run(self, contour: Shape, horizon: Tuple[Tuple[int, int], Tuple[int, int]]) -> (float, Optional[Ellipse]):
        angle = 0.00
        ellipse = None
        if len(contour) >= 5:
            (x, y), (MA, ma), angle = cv2.fitEllipse(np.asarray(contour))
            ellipse = Ellipse((int(x), int(y)), (int(MA/5), int(ma/5)), angle)
            logger.info('Angle from ellipse: %s', (angle - 90) * -1)
            angle = self.angle_processor.compute_angle_for_centroid(x)
            logger.info(ellipse)
        logger.info('angle: %s', angle)
        return angle, ellipse

    def get_inputs_keys(self) -> List[str]:
        return [RoadPart.ROAD_CONTOUR, RoadPart.ROAD_HORIZON]

    def get_outputs_keys(self) -> List[str]:
        return [PILOT_ANGLE, self.ROAD_ELLIPSE]


class RoadEllipseDebugPart(Part):
    IMG_ROAD_ELLIPSE = 'img/road_ellipse'

    def run(self, img: ndarray, road_ellipse: Ellipse) -> ndarray:
        if not road_ellipse:
            return img
        img_debug = cv2.ellipse(img.copy(), center=road_ellipse.center, axes=road_ellipse.axes,
                                angle=road_ellipse.angle, startAngle=0, endAngle=360, color=(20, 255, 100),
                                thickness=2)
        img_debug = cv2.circle(img_debug.copy(), center=road_ellipse.center, radius=5, color=(255, 0, 0))
        return img_debug

    def get_inputs_keys(self) -> List[str]:
        return [RoadDebugPart.IMG_ROAD, AngleRoadPart.ROAD_ELLIPSE]

    def get_outputs_keys(self) -> List[str]:
        return [RoadEllipseDebugPart.IMG_ROAD_ELLIPSE]



