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

    def __init__(self, image_resolution=(120, 160), out_zone_in_percent=20, central_zone_in_percent=20):
        self._out_in_percent = out_zone_in_percent
        self._central_zone_in_percent = central_zone_in_percent
        self._resolution = image_resolution

    def estimate(self, centroids):
        logger.debug("Angle estimation for centroids: %s", centroids)

        for centroid in centroids:
            return self._compute_angle_for_centroid(centroid[0])

        logger.debug("None line found to process data")
        return 0.0

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


class ThrottleControllerFixedSpeed:

    def __init__(self, throttle_value=0.1):
        self._throttle = throttle_value

    def run(self, centroids):
        return self._throttle


class ThresholdController:
    def __init__(self, threshold_limit=160, debug=False):
        self._threshold_limit = threshold_limit
        self._crop_from_top = 20
        self._debug = debug
        self._cache = None

    def shutdown(self):
        pass

    def cache_value(self):
        return self._cache

    def run(self, image_array):
        try:
            img = self._threshold(image_array)
            # img = self._hide_top(img)
            self._cache = img
            return img
        except Exception:
            import numpy
            logging.exception("Unexpected error")
            return self._cache

    def _threshold(self, img):
        limit = self._threshold_limit
        img_gray = cv2.cvtColor(img.copy(), cv2.COLOR_RGB2GRAY)
        (_, binary) = cv2.threshold(img_gray, limit, 255, 0, cv2.THRESH_BINARY)
        return binary

    def _hide_top(self, img):
        (rows, cols) = img.shape
        min_rows = self._crop_from_top
        if self._debug:
            cv2.rectangle(img, (0, 0), (cols, min_rows), (0,), -1)

        return img


class ContourController:
    def __init__(self, debug=False):
        self._debug = debug
        self._cache = None

    def shutdown(self):
        pass

    def cache_value(self):
        return self._cache

    def run(self, image_array):
        try:
            img, contours = self._process_contours(image_array)
            self._cache = img
            return img, contours
        except Exception:
            import numpy
            logging.exception("Unexpected error")
            return self._cache, []

    def _process_contours(self, img_gray):
        (_, cntrs, _) = self._search_geometry(img_gray)

        # Order from bottom to
        if len(cntrs) > 1:
            (cntrs, _) = contours.sort_contours(cntrs, method='bottom-to-top')

        shapes = []
        centroids = []
        img = cv2.cvtColor(img_gray.copy(), cv2.COLOR_GRAY2RGB)
        for contour in cntrs:
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.05 * peri, True)

            if len(approx) < 4:
                continue

            shapes.append(approx)

            moment = cv2.moments(contour)
            mzero = moment['m00']
            if mzero == 0.0:
                mzero = 0.000001
            cx = int(moment['m10'] / mzero)
            cy = int(moment['m01'] / mzero)
            centroids.append((cx, cy))

            cv2.circle(img, (cx, cy), 3, (0, 100, 100), 1)

        cv2.drawContours(img, shapes, -1, (240, 40, 100), 1)

        # First item is probably good item
        cv2.drawContours(img, shapes[0:1], -1, (240, 0, 0), 3)

        logger.debug("Centroids founds: %s", centroids)
        return img, centroids

    @staticmethod
    def _search_geometry(img_gray):
        return cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


class ImagePilot:

    def __init__(self, angle_estimator=AngleProcessorMiddleLine(),
                 throttle_controller=ThrottleControllerFixedSpeed()):
        self._angle_estimator = angle_estimator
        self._throttle_controller = throttle_controller

    def shutdown(self):
        pass

    def run(self, centroids):
        try:
            throttle = self._throttle_controller.run(centroids)
            return self._angle_estimator.estimate(centroids=centroids), throttle
        except Exception:
            logging.exception("Unexpected error")
            return 0.0, 0.0
