import logging
from typing import List, Tuple

import cv2
import numpy
from imutils import contours
from numpy.core.multiarray import ndarray
from paho.mqtt.client import Client, MQTTMessage

from donkeycar.parts.camera import CAM_IMAGE
from donkeycar.parts.mqtt import MqttController
from donkeycar.parts.part import Part

CONTOURS_CENTROIDS = 'contours/centroids'
CONTOURS_SHAPES = 'contours/shapes'

IMG_CONTOURS = 'img/contours'

CFG_THREHOLD_VALUE_ESTIMATOR_CENTROID_VALUE = 'cfg/threshold_value_estimator/centroid_value'

IMG_PROCESSED = 'img/processed'

IMG_GRAY = 'img/gray'

CFG_THRESHOLD_FROM_LINE = 'cfg/threshold/from_line'

CFG_THRESHOLD_DYNAMIC_DELTA = 'cfg/threshold/dynamic/delta'

CFG_THRESHOLD_DYNAMIC_DEFAULT = 'cfg/threshold/dynamic/default'

CFG_THRESHOLD_DYNAMIC_ENABLED = 'cfg/threshold/dynamic/enabled'

CFG_THRESHOLD_LIMIT_MAX = 'cfg/threshold/limit/max'

CFG_THRESHOLD_LIMIT_MIN = 'cfg/threshold/limit/min'

CFG_CONTOURS_ARC_LENGTH_MAX = 'cfg/contours/arc_length_max'
CFG_CONTOURS_ARC_LENGTH_MIN = 'cfg/contours/arc_length_min'
CFG_CONTOURS_POLY_DP_MAX = 'cfg/contours/poly_dp_max'
CFG_CONTOURS_POLY_DP_MIN = 'cfg/contours/poly_dp_min'

logger = logging.getLogger(__name__)

Centroid = Tuple[int, int]
Shape = List[Tuple[int, int]]


class ContoursConfigController(MqttController):

    def __init__(self, poly_dp_min: int = 4, poly_dp_max: int = 100, arc_length_min: int = 10,
                 arc_length_max: int = 100000,
                 mqtt_enable: bool = True,
                 mqtt_topic: str = 'config/contours/#', mqtt_hostname: str = 'localhost', mqtt_port: int = 1883,
                 mqtt_client_id: str = "donkey-config-contours-", mqtt_username: str = None, mqtt_password: str = None,
                 mqtt_qos: int = 0):
        super().__init__(mqtt_client_id, mqtt_enable, mqtt_hostname, mqtt_password, mqtt_port, mqtt_qos, mqtt_topic,
                         mqtt_username, on_message=_on_contours_config_message)
        self.poly_dp_min = poly_dp_min
        self.poly_dp_max = poly_dp_max
        self.arc_length_min = arc_length_min
        self.arc_length_max = arc_length_max

    def run(self) -> (int, int, int, int):
        """
        :return: parts
            * cfg/contours/poly_dp_min
            * cfg/contours/poly_dp_max
            * cfg/contours/arc_length_min
            * cfg/contours/arc_length_max
        """
        return self.poly_dp_min, self.poly_dp_max, self.arc_length_min, self.arc_length_max

    def get_inputs_keys(self) -> List[str]:
        return []

    def get_outputs_keys(self) -> List[str]:
        return [CFG_CONTOURS_POLY_DP_MIN,
                CFG_CONTOURS_POLY_DP_MAX,
                CFG_CONTOURS_ARC_LENGTH_MIN,
                CFG_CONTOURS_ARC_LENGTH_MAX]


def _on_contours_config_message(_: Client, userdata: ContoursConfigController, msg: MQTTMessage):
    logger.info('new message: %s', msg.topic)
    if msg.topic.endswith("contours/poly_dp_min"):
        new_value = int(msg.payload)
        logger.info("Update contours poly_dp_min from %s to %s", userdata.poly_dp_min, new_value)
        userdata.poly_dp_min = new_value
    elif msg.topic.endswith("contours/poly_dp_max"):
        new_value = int(msg.payload)
        logger.info("Update contours poly_dp_max from %s to %s", userdata.poly_dp_max, new_value)
        userdata.poly_dp_max = new_value
    elif msg.topic.endswith("contours/arc_length_min"):
        new_value = int(msg.payload)
        logger.info("Update contours arc_length_min from %s to %s", userdata.arc_length_min, new_value)
        userdata.arc_length_min = new_value
    elif msg.topic.endswith("contours/arc_length_max"):
        new_value = int(msg.payload)
        logger.info("Update contours arc_length_max from %s to %s", userdata.arc_length_max, new_value)
        userdata.arc_length_max = new_value
    else:
        logger.warning("Unexpected msg for topic %s", msg.topic)


class ContoursDetector:
    """
    Search patterns in gray image and extract centroid coordinates matching
    """

    def __init__(self, config: ContoursConfigController = ContoursConfigController(mqtt_enable=False)):
        self._config = config

    def process_image(self, img_binarized: ndarray) -> (List[Shape], List[Centroid]):
        (_, cntrs, _) = self._search_geometry(img_binarized)

        # Order from bottom to
        if len(cntrs) > 1:
            (cntrs, _) = contours.sort_contours(cntrs, method='bottom-to-top')

        shapes = []
        centroids = []

        for contour in cntrs:
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.05 * peri, True)

            if len(approx) < self._config.poly_dp_min \
                    or len(approx) > self._config.poly_dp_max \
                    or peri < self._config.arc_length_min \
                    or peri > self._config.arc_length_max:
                continue

            shapes.append(approx)

            cx, cy = self._compute_centroid(contour)
            centroids.append((cx, cy))
        return shapes, centroids

    @staticmethod
    def _compute_centroid(contour: Shape) -> (int, int):
        moment = cv2.moments(contour)
        mzero = moment['m00']
        if mzero == 0.0:
            mzero = 0.000001
        cx = int(moment['m10'] / mzero)
        cy = int(moment['m01'] / mzero)
        return cx, cy

    @staticmethod
    def _search_geometry(img_gray: ndarray):
        return cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


class ThresholdConfigController(MqttController):

    def __init__(self, limit_min: int, limit_max: int, threshold_dynamic: bool, threshold_default: int,
                 threshold_delta: int, mqtt_enable: bool = True,
                 mqtt_topic: str = 'config/threshold/#', mqtt_hostname: str = 'localhost', mqtt_port: int = 1883,
                 mqtt_client_id: str = "donkey-config-threshold-", mqtt_username: str = None, mqtt_password: str = None,
                 mqtt_qos: int = 0):
        super().__init__(mqtt_client_id, mqtt_enable, mqtt_hostname, mqtt_password, mqtt_port, mqtt_qos, mqtt_topic,
                         mqtt_username, on_message=_on_threshold_config_message)
        self.limit_min = limit_min
        self.limit_max = limit_max
        self.dynamic_enabled = threshold_dynamic
        self.dynamic_default = threshold_default
        self.dynamic_delta = threshold_delta

    def run(self, threshold_from_line: int) -> (int, int, bool, int, int):
        """
        :return: parts
            * cfg/threshold/limit/min
            * cfg/threshold/limit/max
            * cfg/threshold/dynamic/enabled
            * cfg/threshold/dynamic/default
            * cfg/threshold/dynamic/delta
        """
        if self.dynamic_enabled:
            self.dynamic_default = threshold_from_line
            self.limit_min = self.dynamic_default - self.dynamic_delta
            self.limit_max = self.dynamic_default + self.dynamic_delta

        return self.limit_min, self.limit_max, self.dynamic_enabled, self.dynamic_default, self.dynamic_delta

    def get_inputs_keys(self) -> List[str]:
        return [CFG_THRESHOLD_FROM_LINE]

    def get_outputs_keys(self) -> List[str]:
        return [CFG_THRESHOLD_LIMIT_MIN,
                CFG_THRESHOLD_LIMIT_MAX,
                CFG_THRESHOLD_DYNAMIC_ENABLED,
                CFG_THRESHOLD_DYNAMIC_DEFAULT,
                CFG_THRESHOLD_DYNAMIC_DELTA]


class ThresholdController(Part):
    """
    Apply threshold process to gray images
    """

    def __init__(self, config: ThresholdConfigController):
        self._config = config
        self._crop_from_top = 20
        self._video_frame = None

    def run(self, image_gray: ndarray) -> ndarray:
        try:
            img = self._threshold(image_gray)
            self._video_frame = img
            return img
        except Exception:
            import numpy
            logging.exception("Unexpected error")
            return self._video_frame

    def _threshold(self, img: ndarray) -> ndarray:
        (_, binary_min) = cv2.threshold(img.copy(), self._config.limit_min, 255, 0, cv2.THRESH_BINARY)
        (_, binary_max) = cv2.threshold(img.copy(), self._config.limit_max, 255, 0, cv2.THRESH_BINARY_INV)
        return cv2.bitwise_xor(src1=binary_min, src2=binary_max)

    def get_inputs_keys(self) -> List[str]:
        return [IMG_GRAY]

    def get_outputs_keys(self) -> List[str]:
        return [IMG_PROCESSED]


class ThresholdValueEstimatorConfig(MqttController):

    def __init__(self, centroid_value=190,
                 mqtt_enable: bool = True,
                 mqtt_topic: str = 'config/threshold_value_estimator/#', mqtt_hostname: str = 'localhost',
                 mqtt_port: int = 1883, mqtt_client_id: str = "donkey-config-threshold_value_estimator-",
                 mqtt_username: str = None, mqtt_password: str = None, mqtt_qos: int = 0):
        super().__init__(mqtt_client_id, mqtt_enable, mqtt_hostname, mqtt_password, mqtt_port, mqtt_qos, mqtt_topic,
                         mqtt_username, on_message=on_message_threshold_value_estimator)
        self.centroid_value = centroid_value

    def run(self):
        """
        :return: parts:
          * cfg/threshold_value_estimator/centroid_value
        """
        return self.centroid_value

    def get_inputs_keys(self) -> List[str]:
        return []

    def get_outputs_keys(self) -> List[str]:
        return [CFG_THREHOLD_VALUE_ESTIMATOR_CENTROID_VALUE]


def on_message_threshold_value_estimator(_: Client, userdata: ThresholdValueEstimatorConfig, msg: MQTTMessage):
    logger.info('new message: %s', msg.topic)
    if msg.topic.endswith("threshold_value_estimator/centroid_value"):
        new_value = int(msg.payload)
        logger.info("Update contours thresh value_estimator value from %s to %s", userdata.centroid_value, new_value)
        userdata.centroid_value = new_value
    else:
        logger.warning("Unexpected msg for topic %s", msg.topic)


class ThresholdValueEstimator(Part):
    """
    Threshold estimation on gray image. Use near centroid to find pixel value
    """

    def __init__(self, config=ThresholdValueEstimatorConfig(centroid_value=190, mqtt_enable=False),
                 contours_detector=ContoursDetector()):
        self._config = config
        self._video_frame = None
        self._contours_detector = contours_detector

    def run(self, img_gray: ndarray) -> int:
        try:
            (_, binary) = cv2.threshold(img_gray.copy(), self._config.centroid_value, 255, 0, cv2.THRESH_BINARY)
            (shapes, centroids) = self._contours_detector.process_image(img_binarized=binary)

            if not centroids:
                return self._config.centroid_value

            value = img_gray.item((centroids[0][1], centroids[0][0]))
            self._config.centroid_value = value
            logger.debug("Threshold value estimate: %s", value)

            self.draw_image_debug(centroids[0], img_gray, [shapes[0]], value)
            return value
        except Exception:
            import numpy
            logging.exception("Unexpected error")
            return self._config.centroid_value

    def draw_image_debug(self, centroid: Centroid, img_gray: ndarray, shape: Shape, value: int) -> ndarray:
        img_debug = cv2.cvtColor(img_gray.copy(), cv2.COLOR_GRAY2RGB)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img_debug, str(value), (20, 20), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.circle(img_debug, centroid, 3, (0, 100, 100), 1)
        cv2.drawContours(img_debug, shape, -1, (240, 40, 100), 1)
        self._video_frame = img_debug
        return img_debug

    def get_inputs_keys(self) -> List[str]:
        return [IMG_GRAY]

    def get_outputs_keys(self) -> List[str]:
        return [CFG_THRESHOLD_FROM_LINE]


def _on_threshold_config_message(_: Client, userdata: ThresholdConfigController, msg: MQTTMessage):
    logger.info('new message: %s', msg.topic)
    if msg.topic.endswith("threshold/min"):
        new_limit = int(msg.payload)
        logger.info("Update threshold min limit from %s to %s", userdata.limit_min, new_limit)
        userdata.limit_min = new_limit
    elif msg.topic.endswith("threshold/max"):
        new_limit = int(msg.payload)
        logger.info("Update threshold max limit from %s to %s", userdata.limit_max, new_limit)
        userdata.limit_max = new_limit
    elif msg.topic.endswith("threshold/delta"):
        new_value = int(msg.payload)
        logger.info("Update threshold delta from %s to %s", userdata.dynamic_delta, new_value)
        userdata.dynamic_delta = new_value
    elif msg.topic.endswith("threshold/default"):
        new_value = int(msg.payload)
        logger.info("Update threshold default from %s to %s", userdata.dynamic_default, new_value)
        userdata.dynamic_default = new_value
    elif msg.topic.endswith("threshold/dynamic_enabled"):
        new_value = ("true" == msg.payload.decode('utf-8').lower())
        logger.info("Update threshold dynamic enabled from %s to %s (%s)", userdata.dynamic_enabled, new_value,
                    msg.payload)
        userdata.dynamic_enabled = new_value
    else:
        logger.warning("Unexpected msg for topic %s", msg.topic)


class ConvertToGrayPart(Part):
    """
    Convert color image to gray
    """

    def run(self, image_array: ndarray) -> ndarray:
        try:
            return cv2.cvtColor(image_array.copy(), cv2.COLOR_RGB2GRAY)
        except Exception:
            logging.exception("Unexpected error")
            return numpy.zeros(image_array.shape)

    def get_inputs_keys(self) -> List[str]:
        return [CAM_IMAGE]

    def get_outputs_keys(self) -> List[str]:
        return [IMG_GRAY]


class ContourController(Part):

    def __init__(self, contours_detector: ContoursDetector):
        self._video_frame = None
        self._contours_detector = contours_detector

    def run(self, image_array: ndarray) -> (ndarray, List[Shape], List[Centroid]):
        try:
            img, shapes, centroids = self._process_contours(image_array)
            self._video_frame = img
            return img, shapes, centroids
        except Exception:
            import numpy
            logging.exception("Unexpected error")
            return self._video_frame, []

    def _process_contours(self, img_gray: ndarray) -> (ndarray, List[Centroid]):
        shapes, centroids = self._contours_detector.process_image(img_gray)

        img = cv2.cvtColor(img_gray.copy(), cv2.COLOR_GRAY2RGB)
        for centroid in centroids:
            cv2.circle(img, centroid, 3, (0, 100, 100), 1)

        cv2.drawContours(img, shapes, -1, (240, 40, 100), 1)

        logger.debug("Centroids founds: %s", centroids)
        return img, shapes, centroids

    def get_inputs_keys(self) -> List[str]:
        return [IMG_PROCESSED]

    def get_outputs_keys(self) -> List[str]:
        return [IMG_CONTOURS,
                CONTOURS_SHAPES,
                CONTOURS_CENTROIDS]
