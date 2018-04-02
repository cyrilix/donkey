import logging

import cv2
from imutils import contours

logger = logging.getLogger(__name__)


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

    def __init__(self, image_resolution=(120, 160), out_zone_in_percent=20, central_zone_in_percent=20,
                 use_only_first=False):
        self._out_in_percent = out_zone_in_percent
        self._central_zone_in_percent = central_zone_in_percent
        self._resolution = image_resolution
        self._last_value = 0
        self._use_only_first = use_only_first

    def estimate(self, centroids):
        logger.debug("Angle estimation for centroids: %s", centroids)

        if not centroids:
            logger.debug("None line found to process data")
            return self._last_value

        if len(centroids) == 1:
            angle = self._compute_angle_for_centroid(centroids[0][0])
        else:
            x_values = centroids[0][0]
            nb_values = 1

            if len(centroids) >= 2 and not self._use_only_first:
                x_values += centroids[1][0]
                nb_values += 1

            if len(centroids) >= 4 and not self._use_only_first:
                x_values += centroids[2][0]

            angle = self._compute_angle_for_centroid(x_values / nb_values)

        if angle < 0:
            self._last_value = -1
        if angle > 0:
            self._last_value = 1
        return angle

    def _compute_angle_for_centroid(self, line):
        # Position in percent from the left of the middle line
        pos_in_percent = line * 100 / self._resolution[1]
        logger.debug("Line position from left = %s%% (cx=%s, resolution=%s)", pos_in_percent, line, self._resolution[1])

        # convert between -1 and 1
        angle = (pos_in_percent * 2 - 100) / 100

        logger.debug("Computed angle: %s", angle)
        out_zone_delta = self._out_in_percent * 100 / self._resolution[1] / 100
        logger.debug("Outer zone delta: %s", out_zone_delta)
        middle_zone_delta = self._central_zone_in_percent * 100 / self._resolution[1] / 100
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


class ConvertToGrayPart:
    """
    Convert color image to gray
    """

    @staticmethod
    def run(image_array):
        try:
            return cv2.cvtColor(image_array.copy(), cv2.COLOR_RGB2GRAY)
        except Exception:
            logging.exception("Unexpected error")
            return None

    def shutdown(self):
        pass


class ThrottleControllerFixedSpeed:

    def __init__(self, throttle_value=0.1):
        self._throttle = throttle_value
        self._shock = False

    def shutdown(self):
        pass

    def run(self, centroids, shock):
        if shock:
            logger.info("!!!!!!! SHOCK DETECTED !!!!!!!!")
            self._shock = shock

        if self._shock:
            return 0.0
        else:
            return self._throttle


class ThrottleControllerSteeringBased:
    """
    Implementation of throttle controller using steering value
    """

    def __init__(self, min_speed=0.1, max_speed=1.0, safe_angle=0.2, dangerous_angle=0.8):
        self._min_speed = min_speed
        self._max_speed = max_speed
        self._dangerous_angle = dangerous_angle
        self._safe_angle = safe_angle
        self._shock = False

    def shutdown(self):
        pass

    def run(self, angle, shock):
        if shock:
            logger.info("!!!!!!! SHOCK DETECTED !!!!!!!!")
            self._shock = shock

        if self._shock:
            return 0.0

        # Angle between 0 - safe direction ==> max_speed
        if angle < self._safe_angle:
            return self._max_speed
        # Angle > danger => min speed
        if angle > self._dangerous_angle:
            return self._min_speed

        # other ==> proportional to (max_speed - min_speed )
        speed_interv = self._max_speed - self._min_speed
        return round((angle * speed_interv) + self._min_speed, 2)


class ThresholdController:
    """
    Apply threshold process to gray images
    """

    def __init__(self, threshold_defaut=190, threshold_delta=10, debug=False):
        self._crop_from_top = 20
        self._debug = debug
        self._video_frame = None
        self._threshold_default = threshold_defaut
        self._threshold_delta = threshold_delta

    def shutdown(self):
        pass

    def video_frame(self):
        return self._video_frame

    def run(self, image_gray, threshold_value):
        try:
            img = self._threshold(image_gray, threshold_value)
            # img = self._hide_top(img)
            self._video_frame = img
            return img
        except Exception:
            import numpy
            logging.exception("Unexpected error")
            return self._video_frame

    def _threshold(self, img, threshold_value):
        (_, binary_min) = cv2.threshold(img.copy(), (threshold_value - self._threshold_delta), 255, 0,
                                        cv2.THRESH_BINARY)
        (_, binary_max) = cv2.threshold(img.copy(), (threshold_value + self._threshold_delta), 255, 0,
                                        cv2.THRESH_BINARY_INV)
        return cv2.bitwise_xor(src1=binary_min, src2=binary_max)

    def _hide_top(self, img):
        (rows, cols) = img.shape
        min_rows = self._crop_from_top
        if self._debug:
            cv2.rectangle(img, (0, 0), (cols, min_rows), (0,), -1)

        return img


class ThresholdValueEstimator:
    """
    Threshold estimation on gray image. Use near centroid to find pixel value
    """

    def __init__(self, init_value=190, contours_detector=None):

        self._init_value = init_value
        self._value = init_value
        self._video_frame = None
        if contours_detector:
            self._contours_detector = contours_detector
        else:
            self._contours_detector = ContoursDetector()

    def shutdown(self):
        pass

    def run(self, img_gray):
        try:
            (_, binary) = cv2.threshold(img_gray.copy(), self._value, 255, 0, cv2.THRESH_BINARY)
            (shapes, centroids) = self._contours_detector.process_image(img_binarized=binary)

            if not centroids:
                return self._init_value

            value = img_gray.item((centroids[0][1], centroids[0][0]))
            self._value = value
            logger.debug("Threshold value estimate: %s", value)

            self.draw_image_debug(centroids[0], img_gray, [shapes[0]], value)
            return value
        except Exception:
            import numpy
            logging.exception("Unexpected error")
            return self._init_value

    def video_frame(self):
        return self._video_frame

    def draw_image_debug(self, centroids, img_gray, shape, value):
        img_debug = cv2.cvtColor(img_gray.copy(), cv2.COLOR_GRAY2RGB)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img_debug, str(value), (20, 20), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.circle(img_debug, centroids, 3, (0, 100, 100), 1)
        cv2.drawContours(img_debug, shape, -1, (240, 40, 100), 1)
        self._video_frame = img_debug
        return img_debug


class ContoursDetector:
    """
    Search patterns in gray image and extract centroid coordinates matching
    """

    def __init__(self, poly_dp_min=4, arc_length_min=10, arc_length_max=100000):
        self._poly_dp_min = poly_dp_min
        self._arc_length_min = arc_length_min
        self._arc_length_max = arc_length_max

    def process_image(self, img_binarized):
        (_, cntrs, _) = self._search_geometry(img_binarized)

        # Order from bottom to
        if len(cntrs) > 1:
            (cntrs, _) = contours.sort_contours(cntrs, method='bottom-to-top')

        shapes = []
        centroids = []

        for contour in cntrs:
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.05 * peri, True)

            if len(approx) < self._poly_dp_min or peri < self._arc_length_min or peri > self._arc_length_max:
                continue

            shapes.append(approx)

            cx, cy = self._compute_centroid(contour)
            centroids.append((cx, cy))
        return shapes, centroids

    @staticmethod
    def _compute_centroid(contour):
        moment = cv2.moments(contour)
        mzero = moment['m00']
        if mzero == 0.0:
            mzero = 0.000001
        cx = int(moment['m10'] / mzero)
        cy = int(moment['m01'] / mzero)
        return cx, cy

    @staticmethod
    def _search_geometry(img_gray):
        return cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


class ContourController:
    def __init__(self, contours_detector, debug=False):
        self._debug = debug
        self._video_frame = None
        self._contours_detector = contours_detector

    def shutdown(self):
        pass

    def video_frame(self):
        return self._video_frame

    def run(self, image_array):
        try:
            img, centroids = self._process_contours(image_array)
            self._video_frame = img
            return img, centroids
        except Exception:
            import numpy
            logging.exception("Unexpected error")
            return self._video_frame, []

    def _process_contours(self, img_gray):
        shapes, centroids = self._contours_detector.process_image(img_gray)

        img = cv2.cvtColor(img_gray.copy(), cv2.COLOR_GRAY2RGB)
        for centroid in centroids:
            cv2.circle(img, centroid, 3, (0, 100, 100), 1)

        cv2.drawContours(img, shapes, -1, (240, 40, 100), 1)

        # First item is probably good item
        cv2.drawContours(img, shapes[0:1], -1, (240, 0, 0), 3)

        logger.debug("Centroids founds: %s", centroids)
        return img, centroids


class ImagePilot:
    def __init__(self, angle_estimator=AngleProcessorMiddleLine(),
                 throttle_controller=ThrottleControllerFixedSpeed()):
        self._angle_estimator = angle_estimator
        self._throttle_controller = throttle_controller

    def shutdown(self):
        pass

    def run(self, centroids, shock):
        try:
            angle = self._angle_estimator.estimate(centroids=centroids)
            throttle = self._throttle_controller.run(angle, shock)
            return angle, throttle
        except Exception:
            logging.exception("Unexpected error")
            return 0.0, 0.0
