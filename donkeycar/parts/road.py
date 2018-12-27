import logging
from collections import namedtuple
from typing import List, Tuple

import cv2
import numpy as np
from imutils import contours
from numpy import ndarray
from paho.mqtt.client import Client, MQTTMessage

from donkeycar.parts.camera import CAM_IMAGE
from donkeycar.parts.img_process import IMG_GRAY, ConvertToGrayPart, BoundingBoxPart, HistogramPart, ThresholdPart, \
    BlurPart, CannyPart
from donkeycar.parts.mqtt import MqttController
from donkeycar.parts.part import Part
from donkeycar.parts.threshold import Shape

CFG_ROAD_ENABLE = "cfg/road/enable"
CFG_ROAD_HORIZON_HOUGH_MIN_LINE_LENGTH = "cfg/road/horizon/hough_min_line_length"
CFG_ROAD_HORIZON_HOUGH_MAX_LINE_GAP = "cfg/road/horizon/hough_max_line_gap"
CFG_ROAD_HORIZON_HOUGH_THRESHOLD = "cfg/road/horizon/hough_threshold"
CFG_ROAD_CONTOUR_KERNEL_SIZE = "cfg/road/contour/kernel_size"
CFG_ROAD_CONTOUR_MORPHO_ITERATIONS = "cfg/road/contour/morpho_iterations"
CFG_ROAD_CONTOUR_CANNY_THRESHOLD1 = "cfg/road/contour/canny_threshold1"
CFG_ROAD_CONTOUR_CANNY_THRESHOLD2 = "cfg/road/contour/canny_threshold2"
CFG_ROAD_CONTOUR_APPROX_POLY_EPSILON_FACTOR = "cfg/road/contour/approx_poly_epsilon_factor"

logger = logging.getLogger(__name__)


class RoadConfigController(MqttController):

    def __init__(self,
                 enable=False,
                 horizon_hough_min_line_length: int = 1,
                 horizon_hough_max_line_gap: int = 10,
                 horizon_hough_threshold: int = 100,
                 kernel_size: int = 4,
                 morpho_iterations: int = 3,
                 canny_threshold1: int = 120,
                 canny_threshold2: int = 250,
                 approx_poly_epsilon_factor: float = 0.01,
                 mqtt_enable: bool = True, mqtt_topic: str = 'config/road/#', mqtt_hostname: str = 'localhost',
                 mqtt_port: int = 1883, mqtt_client_id: str = "donkey-config-road-", mqtt_username: str = None,
                 mqtt_password: str = None, mqtt_qos: int = 0):
        super().__init__(mqtt_client_id, mqtt_enable, mqtt_hostname, mqtt_password, mqtt_port, mqtt_qos, mqtt_topic,
                         mqtt_username, on_message=_on_road_config_message)
        self.enable = enable
        self.horizon_hough_minLineLength = horizon_hough_min_line_length
        self.horizon_hough_maxLineGap = horizon_hough_max_line_gap
        self.horizon_hough_threshold = horizon_hough_threshold
        self.kernel_size = kernel_size
        self.morpho_iterations = morpho_iterations
        self.canny_threshold1 = canny_threshold1
        self.canny_threshold2 = canny_threshold2
        self.approxPoly_epsilon_factor = approx_poly_epsilon_factor

    def run(self) -> (int, int, int, int, int, int, int, float):
        """
        :return: parts
            * cfg/road/enable
            * cfg/road/horizon/hough_min_line_length
            * cfg/road/horizon/hough_max_line_gap
            * cfg/road/horizon/hough_threshold
            * cfg/road/contour/kernel_size
            * cfg/road/contour/morpho_iterations
            * cfg/road/contour/canny_threshold1
            * cfg/road/contour/canny_threshold2
            * cfg/road/contour/approx_poly_epsilon_factor
        """
        return self.enable, self.horizon_hough_minLineLength, self.horizon_hough_maxLineGap, \
               self.horizon_hough_threshold, self.kernel_size, self.morpho_iterations, self.canny_threshold1, \
               self.canny_threshold2, self.approxPoly_epsilon_factor

    def get_inputs_keys(self) -> List[str]:
        return []

    def get_outputs_keys(self) -> List[str]:
        return [CFG_ROAD_ENABLE,
                CFG_ROAD_HORIZON_HOUGH_MIN_LINE_LENGTH,
                CFG_ROAD_HORIZON_HOUGH_MAX_LINE_GAP,
                CFG_ROAD_HORIZON_HOUGH_THRESHOLD,
                CFG_ROAD_CONTOUR_KERNEL_SIZE,
                CFG_ROAD_CONTOUR_MORPHO_ITERATIONS,
                CFG_ROAD_CONTOUR_CANNY_THRESHOLD1,
                CFG_ROAD_CONTOUR_CANNY_THRESHOLD2,
                CFG_ROAD_CONTOUR_APPROX_POLY_EPSILON_FACTOR]


def _on_road_config_message(_: Client, userdata: RoadConfigController, msg: MQTTMessage) -> None:
    logger.info('new message: %s', msg.topic)
    if msg.topic.endswith(CFG_ROAD_ENABLE.replace("cfg/", '')):
        new_value = ("true" == msg.payload.decode('utf-8').lower())
        logger.info("Update %s from %s to %s",
                    CFG_ROAD_ENABLE, userdata.enable, new_value)
        userdata.enable = new_value
    elif msg.topic.endswith(CFG_ROAD_HORIZON_HOUGH_MIN_LINE_LENGTH.replace("cfg/", '')):
        new_value = int(msg.payload)
        logger.info("Update %s from %s to %s",
                    CFG_ROAD_HORIZON_HOUGH_MIN_LINE_LENGTH, userdata.horizon_hough_minLineLength, new_value)
        userdata.horizon_hough_minLineLength = new_value
    elif msg.topic.endswith(CFG_ROAD_HORIZON_HOUGH_MAX_LINE_GAP.replace("cfg/", '')):
        new_value = int(msg.payload)
        logger.info("Update %s from %s to %s",
                    CFG_ROAD_HORIZON_HOUGH_MAX_LINE_GAP, userdata.horizon_hough_maxLineGap, new_value)
        userdata.horizon_hough_maxLineGap = new_value
    elif msg.topic.endswith(CFG_ROAD_HORIZON_HOUGH_THRESHOLD.replace("cfg/", '')):
        new_value = int(msg.payload)
        logger.info("Update %s from %s to %s",
                    CFG_ROAD_HORIZON_HOUGH_THRESHOLD, userdata.horizon_hough_threshold, new_value)
        userdata.horizon_hough_threshold = new_value
    elif msg.topic.endswith(CFG_ROAD_CONTOUR_KERNEL_SIZE.replace("cfg/", '')):
        new_value = int(msg.payload)
        logger.info("Update %s from %s to %s",
                    CFG_ROAD_CONTOUR_KERNEL_SIZE, userdata.kernel_size, new_value)
        userdata.kernel_size = new_value
    elif msg.topic.endswith(CFG_ROAD_CONTOUR_MORPHO_ITERATIONS.replace("cfg/", '')):
        new_value = int(msg.payload)
        logger.info("Update %s from %s to %s",
                    CFG_ROAD_CONTOUR_MORPHO_ITERATIONS, userdata.morpho_iterations, new_value)
        userdata.morpho_iterations = new_value
    elif msg.topic.endswith(CFG_ROAD_CONTOUR_CANNY_THRESHOLD1.replace("cfg/", '')):
        new_value = int(msg.payload)
        logger.info("Update %s from %s to %s",
                    CFG_ROAD_CONTOUR_CANNY_THRESHOLD1, userdata.canny_threshold1, new_value)
        userdata.canny_threshold1 = new_value
    elif msg.topic.endswith(CFG_ROAD_CONTOUR_CANNY_THRESHOLD2.replace("cfg/", '')):
        new_value = int(msg.payload)
        logger.info("Update %s from %s to %s",
                    CFG_ROAD_CONTOUR_CANNY_THRESHOLD2, userdata.canny_threshold2, new_value)
        userdata.canny_threshold2 = new_value
    elif msg.topic.endswith(CFG_ROAD_CONTOUR_APPROX_POLY_EPSILON_FACTOR.replace("cfg/", '')):
        new_value = float(msg.payload)
        logger.info("Update %s from %s to %s",
                    CFG_ROAD_CONTOUR_APPROX_POLY_EPSILON_FACTOR, userdata.approxPoly_epsilon_factor, new_value)
        userdata.approxPoly_epsilon_factor = new_value
    else:
        logger.warning("Unexpected msg for topic %s", msg.topic)


class RoadPart(Part):
    """
    Road detection from binarized
    """

    EMPTY_ROAD_CONTOUR = []
    EMPTY_HORIZON = ((0, 0), (0, 0))

    ROAD_CONTOUR = 'road/contour'
    ROAD_HORIZON = 'road/horizon'

    def __init__(self, config: RoadConfigController = RoadConfigController(mqtt_enable=False), input_img_type=IMG_GRAY):
        self._input_img_type = input_img_type
        self._config = config

    def run(self, img_gray: ndarray, input_img_type=IMG_GRAY) -> (Shape, Tuple[Tuple[int, int]]):
        try:
            if not self._config.enable:
                return self.EMPTY_ROAD_CONTOUR, self.EMPTY_HORIZON

            kernel = np.ones((self._config.kernel_size, self._config.kernel_size), np.uint8)

            # img_final = cv2.Canny(image=img_gray.copy(),
            #                      threshold1=self._config.canny_threshold1,
            #                      threshold2=self._config.canny_threshold2)
            img_final = img_gray.copy()
            start_horizon_line, end_horizon_line = self.search_horizon(img_final)

            img_final = cv2.dilate(img_final, kernel, iterations=self._config.morpho_iterations)
            img_final = cv2.erode(img_final, kernel, iterations=self._config.morpho_iterations)

            img_inversed = np.invert(img_final)

            # Draw black rectangle above horizon
            img_inversed = cv2.rectangle(img=img_inversed, pt1=(0, 0), pt2=end_horizon_line, color=0,
                                         thickness=cv2.FILLED)

            return self._detect_road_contour(img_inversed), (start_horizon_line, end_horizon_line)
        except:
            logging.exception("Unexpected error")
            return self.EMPTY_ROAD_CONTOUR, self.EMPTY_HORIZON

    def search_horizon(self, edges: ndarray) -> (Tuple[int, int], Tuple[int, int]):
        lines = cv2.HoughLinesP(image=edges, rho=4, theta=np.pi / 180, threshold=self._config.horizon_hough_threshold,
                                minLineLength=self._config.horizon_hough_minLineLength,
                                maxLineGap=self._config.horizon_hough_maxLineGap)
        ys = []
        if lines is None:
            return (0, 0), (edges.shape[1], 0)

        for line in lines:
            x1, y1, x2, y2 = line[0]
            if 2 > y1 - y2 > -2:
                ys.append((y1 + y2) / 2)

        import statistics
        if ys:
            y = int(statistics.mean(ys))
        else:
            y = 0
        return (0, y), (edges.shape[1], y)

    def _detect_road_contour(self, img_inversed: ndarray) -> Shape:
        (_, cntrs, _) = cv2.findContours(img_inversed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(cntrs) == 0:
            return []
        elif len(cntrs) == 1:
            (cntrs, _) = contours.sort_contours(cntrs, method='bottom-to-top')
            cntr = cntrs[0]
            epsilon = self._config.approxPoly_epsilon_factor * cv2.arcLength(cntr, True)
        else:
            peris = [cv2.arcLength(c, True) for c in cntrs]
            idx = np.argmax(peris).item()
            epsilon = self._config.approxPoly_epsilon_factor * peris[idx]
            cntr = cntrs[idx]
        approx = cv2.approxPolyDP(cntr, epsilon, True)
        return [(x[0, 0], x[0, 1]) for x in list(approx[:, :])]

    def get_inputs_keys(self) -> List[str]:
        return [self._input_img_type]

    def get_outputs_keys(self) -> List[str]:
        return [RoadPart.ROAD_CONTOUR, RoadPart.ROAD_HORIZON]


Ellipse = namedtuple('Ellipse', ('center', 'axes', 'angle', 'trust'))


class RoadEllipsePart(Part):

    ROAD_ELLIPSE = 'road/ellipse'

    def run(self, contour: Shape) -> Ellipse:
        trust_detection = 1.0
        if len(contour) < 5:
            return Ellipse(None, None, 90.0, 0.0)

        # Check center position
        # Check MA/ma ration
        (x, y), (MA, ma), angle = cv2.fitEllipse(np.asarray(contour))
        ellipse = Ellipse((int(x), int(y)), (MA, ma), angle, trust_detection)

        return ellipse

    def get_inputs_keys(self) -> List[str]:
        return [RoadPart.ROAD_CONTOUR]

    def get_outputs_keys(self) -> List[str]:
        return [RoadEllipsePart.ROAD_ELLIPSE]


class RoadDebugPart(Part):
    IMG_ROAD = "img/road"

    def run(self, road_shape: Shape, horizon: Tuple[Tuple[int, int], Tuple[int, int]], img: ndarray) -> ndarray:
        try:
            if not road_shape:
                return np.zeros(img.shape, dtype=img.dtype)
            mask = np.zeros(img.shape, np.uint8)

            mask = cv2.drawContours(image=mask, contours=[np.array(road_shape)], contourIdx=0,
                                    color=(240, 40, 100), thickness=cv2.FILLED)

            road_img = cv2.addWeighted(src1=img.copy(), alpha=0.7,
                                       src2=mask, beta=0.3,
                                       gamma=0)
            road_img = cv2.line(img=road_img, pt1=horizon[0], pt2=horizon[1], color=(0, 0, 255), thickness=2)
            return road_img
        except:
            logging.exception("Unexpected error")
            return np.zeros(img.shape, dtype=img.dtype)

    def get_inputs_keys(self) -> List[str]:
        return [RoadPart.ROAD_CONTOUR, RoadPart.ROAD_HORIZON, CAM_IMAGE]

    def get_outputs_keys(self) -> List[str]:
        return [RoadDebugPart.IMG_ROAD]


class ComponentRoadPart(Part):

    def __init__(self, input_keys=[CAM_IMAGE]):
        self._input_keys = input_keys
        self._gray_part = ConvertToGrayPart()
        self._bbox_part = BoundingBoxPart(input_img_key='', output_img_key='')
        self._histogram_part = HistogramPart()
        self._gray2_part = ThresholdPart()
        self._blur_part = BlurPart(input_key="", output_key="")
        self._canny_part = CannyPart(input_img_key='', output_img_key='')
        road_config = RoadConfigController(enable=True,
                                           canny_threshold1=180,
                                           canny_threshold2=200,
                                           kernel_size=4,
                                           mqtt_enable=False)
        self._road_part = RoadPart(config=road_config, input_img_type='')
        self._road_debug_part = RoadDebugPart()
        self._road_ellipse_part = RoadEllipsePart()
        self._last_road_contour = None

    def run(self, img: np.ndarray) -> \
            (ndarray, ndarray, ndarray,  ndarray, Shape, Tuple[Tuple[int, int]], ndarray, Ellipse):
        try:
            img_gray = self._gray_part.run(img)
            bbox = self._bbox_part.run(img_gray, road_contour=self._last_road_contour)
            histogram = self._histogram_part.run(bbox)
            gray2 = self._gray2_part.run(histogram)
            blur = self._blur_part.run(gray2)
            canny = self._canny_part.run(blur)
            road_contour, horizon = self._road_part.run(canny)
            if road_contour:
                self._last_road_contour = road_contour
            road_ellipse = self._road_ellipse_part.run(road_contour)
            road_debug = self._road_debug_part.run(road_shape=road_contour, horizon=horizon, img=img)
            return img_gray, gray2, blur, canny, road_contour, horizon, road_debug, road_ellipse
        except:
            logging.exception("Unexpected error")
            return np.zeros(img.shape, dtype=img.dtype)

    def get_inputs_keys(self) -> List[str]:
        return self._input_keys

    def get_outputs_keys(self) -> List[str]:
        return [ConvertToGrayPart.IMG_GRAY_RAW, ThresholdPart.IMG_THRESHOLD, BlurPart.IMG_BLUR, CannyPart.IMG_CANNY,
                RoadPart.ROAD_CONTOUR, RoadPart.ROAD_HORIZON, RoadDebugPart.IMG_ROAD, RoadEllipsePart.ROAD_ELLIPSE]
