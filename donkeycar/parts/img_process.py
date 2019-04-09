import logging
from typing import List, Optional

import cv2
import numpy as np
from numpy import ndarray

from donkeycar.parts.camera import CAM_IMAGE
from donkeycar.parts.part import Part

IMG_GRAY = 'img/gray'
IMG_GRAY_EQUALIZED = 'img/gray/equalized'

logger = logging.getLogger(__name__)


class HistogramPart(Part):
    def __init__(self, clip_limit=2.0, tile_grid_size=(8, 8)):
        self._clip_limit = clip_limit
        self._tile_grid_size = tile_grid_size

    def run(self, img_gray: ndarray) -> Optional[ndarray]:
        try:
            clahe = cv2.createCLAHE(clipLimit=self._clip_limit, tileGridSize=self._tile_grid_size)
            return clahe.apply(img_gray.copy())
        except Exception:
            logging.exception("Unexpected error")
            return None

    def get_inputs_keys(self) -> List[str]:
        return [ConvertToGrayPart.IMG_GRAY_RAW]

    def get_outputs_keys(self) -> List[str]:
        return [IMG_GRAY_EQUALIZED]


class BlurPart(Part):
    IMG_BLUR = 'img/blur'

    def __init__(self, input_key: str, output_key=IMG_BLUR, kernel_size=5):
        self._kernel_size = (kernel_size, kernel_size)
        self._input_keys = [input_key]
        self._output_keys = [output_key]

    def run(self, img: ndarray) -> Optional[ndarray]:
        try:
            return cv2.blur(img.copy(), self._kernel_size)
        except Exception:
            logging.exception("Unexpected error")
            return None

    def get_inputs_keys(self) -> List[str]:
        return self._input_keys

    def get_outputs_keys(self) -> List[str]:
        return self._output_keys


class BoundingBoxPart(Part):

    def __init__(self, input_img_key: str, output_img_key: str):
        self._input_keys = [input_img_key, 'road/contour']
        self._output_keys = [output_img_key]
        self._previous_bb = None

    def run(self, img: ndarray, road_contour) -> Optional[ndarray]:
        try:
            if not road_contour and not self._previous_bb:
                logger.debug('no road')
                return img
            if not road_contour:
                road_contour = self._previous_bb

            logger.debug(road_contour)
            x, y, w, h = cv2.boundingRect(np.array(road_contour))
            if self._previous_bb and (w < 20 or h < 100):
                x, y, w, h = cv2.boundingRect(np.array(self._previous_bb))
            else:
                self._previous_bb = road_contour
            marge = 10
            img_bbox = img.copy()
            if y > 0:
                img_bbox = cv2.rectangle(img=img_bbox, pt1=(0, 0), pt2=(img.shape[1], y - marge),
                                         color=0, thickness=cv2.FILLED)
            if img.shape[1] - (y + h) > 0:
                img_bbox = cv2.rectangle(img=img_bbox, pt1=(0, y + h), pt2=(img.shape[1], img.shape[0]),
                                         color=0, thickness=cv2.FILLED)
            if x > 0:
                img_bbox = cv2.rectangle(img=img_bbox, pt1=(0, 0), pt2=(x, img.shape[0] - marge),
                                         color=0, thickness=cv2.FILLED)
            if img.shape[0] - (x + w) > 0:
                img_bbox = cv2.rectangle(img=img_bbox, pt1=(x + w + marge, 0), pt2=(img.shape[1], img.shape[0] - marge),
                                         color=0, thickness=cv2.FILLED)
            return img_bbox
        except Exception:
            logging.exception("Unexpected error")
            return None

    def get_inputs_keys(self) -> List[str]:
        return self._input_keys

    def get_outputs_keys(self) -> List[str]:
        return self._output_keys


class ConvertToGrayPart(Part):
    """
    Convert color image to gray
    """
    IMG_GRAY_RAW = 'img/gray/raw'

    def run(self, img: ndarray) -> ndarray:
        try:
            return cv2.cvtColor(img.copy(), cv2.COLOR_RGB2GRAY)
        except Exception:
            logging.exception("Unexpected error")
            return np.zeros(img.shape)

    def get_inputs_keys(self) -> List[str]:
        return [CAM_IMAGE]

    def get_outputs_keys(self) -> List[str]:
        return [ConvertToGrayPart.IMG_GRAY_RAW]


class GraySelectorPart(Part):
    def run(self, img_gray_raw, img_gray_equalized):
        return img_gray_equalized

    def get_inputs_keys(self) -> List[str]:
        return [ConvertToGrayPart.IMG_GRAY_RAW, IMG_GRAY_EQUALIZED]

    def get_outputs_keys(self) -> List[str]:
        return [IMG_GRAY]


IMG_DILATED = 'img/dilated'


class DilatePart(Part):
    def __init__(self, input_img_key='img/processed'):
        self._input = input_img_key

    def run(self, img: ndarray):
        kernel = np.ones((3, 3), np.uint8)
        # opening = cv.morphologyEx(img,cv.MORPH_OPEN,kernel, iterations = 2)
        return cv2.dilate(src=img.copy(), kernel=kernel, iterations=2)

    def get_inputs_keys(self) -> List[str]:
        return [self._input]

    def get_outputs_keys(self) -> List[str]:
        return [IMG_DILATED]


class MorphologyOpenPart(Part):
    def __init__(self, input_img_key, output_img_key, kernel_size=5, iterations=2):
        self._input = input_img_key
        self._output_img_key = output_img_key
        self._kernel_size = kernel_size
        self._iterations = iterations
        self._kernel = np.ones((kernel_size, kernel_size), np.uint8)

    def run(self, img: ndarray):
        return cv2.morphologyEx(src=img.copy(), op=cv2.MORPH_OPEN, kernel=self._kernel, iterations=self._iterations)

    def get_inputs_keys(self) -> List[str]:
        return [self._input]

    def get_outputs_keys(self) -> List[str]:
        return [self._output_img_key]


class ThresholdPart(Part):
    IMG_THRESHOLD = 'img/threshold'

    def __init__(self, lower_bound=200, upper_bound=255, input_img_key=IMG_GRAY_EQUALIZED):
        self._lower_bound = lower_bound
        self._upper_bound = upper_bound
        self._input = input_img_key

    def run(self, img: ndarray):
        mask_white = cv2.inRange(img, self._lower_bound, self._upper_bound)
        return mask_white

    @staticmethod
    def _max_value(img: ndarray) -> int:
        n = 5
        center = [slice(i-n, i+n+1) for i in img]
        return np.max(img[72:82, 72:82])

    def get_inputs_keys(self) -> List[str]:
        return [self._input]

    def get_outputs_keys(self) -> List[str]:
        return [ThresholdPart.IMG_THRESHOLD]


class CannyPart(Part):
    IMG_CANNY = 'img/canny'

    def __init__(self, input_img_key, output_img_key=IMG_CANNY, low_threshold=50, high_threshold=150):
        self._input = input_img_key
        self._output = output_img_key
        self._low_threshold = low_threshold
        self._high_threshold = high_threshold

    def run(self, img: ndarray) -> ndarray:
        # TODO: inject threshold
        return cv2.Canny(img, self._low_threshold, self._high_threshold)

    def get_inputs_keys(self) -> List[str]:
        return [self._input]

    def get_outputs_keys(self) -> List[str]:
        return [self._output]


class HoughPart(Part):
    def __init__(self, input_img_key, output_img_key):
        self.cache = 0
        self.first_frame = 0
        self._input = input_img_key
        self._output = output_img_key

    def run(self, img: ndarray):
        # rho and theta are the distance and angular resolution of the grid in Hough space
        # same values as quiz
        rho = 4
        theta = np.pi / 180
        # threshold is minimum number of intersections in a grid for candidate line to go to output
        threshold = 30
        min_line_len = 100
        max_line_gap = 180
        # my hough values started closer to the values in the quiz, but got bumped up considerably for the challenge video
        return self.hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap)

    def hough_lines(self, img, rho, theta, threshold, min_line_len, max_line_gap):
        """
        `img` should be the output of a Canny transform.

        Returns an image with hough lines drawn.
        """
        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len,
                                maxLineGap=max_line_gap)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        self.draw_lines(line_img, lines)
        return line_img

    def draw_lines(self, img, lines, color=[255, 0, 0], thickness=6):
        """workflow:
        1) examine each individual line returned by hough & determine if it's in left or right lane by its slope
        because we are working "upside down" with the array, the left lane will have a negative slope and right positive
        2) track extrema
        3) compute averages
        4) solve for b intercept
        5) use extrema to solve for points
        6) smooth frames and cache
        """
        y_global_min = img.shape[0]  # min will be the "highest" y value, or point down the road away from car
        y_max = img.shape[0]
        l_slope, r_slope = [], []
        l_lane, r_lane = [], []
        det_slope = 0.4
        α = 0.2
        # i got this alpha value off of the forums for the weighting between frames.
        # i understand what it does, but i dont understand where it comes from
        # much like some of the parameters in the hough function

        for line in lines:
            # 1
            for x1, y1, x2, y2 in line:
                slope = self.get_slope(x1, y1, x2, y2)
                if slope > det_slope:
                    r_slope.append(slope)
                    r_lane.append(line)
                elif slope < -det_slope:
                    l_slope.append(slope)
                    l_lane.append(line)
            # 2
            y_global_min = min(y1, y2, y_global_min)

        # to prevent errors in challenge video from dividing by zero
        if ((len(l_lane) == 0) or (len(r_lane) == 0)):
            print('no lane detected')
            return 1

        # 3
        l_slope_mean = np.mean(l_slope, axis=0)
        r_slope_mean = np.mean(r_slope, axis=0)
        l_mean = np.mean(np.array(l_lane), axis=0)
        r_mean = np.mean(np.array(r_lane), axis=0)

        if ((r_slope_mean == 0) or (l_slope_mean == 0)):
            print('dividing by zero')
            return 1

        # 4, y=mx+b -> b = y -mx
        l_b = l_mean[0][1] - (l_slope_mean * l_mean[0][0])
        r_b = r_mean[0][1] - (r_slope_mean * r_mean[0][0])

        # 5, using y-extrema (#2), b intercept (#4), and slope (#3) solve for x using y=mx+b
        # x = (y-b)/m
        # these 4 points are our two lines that we will pass to the draw function
        l_x1 = int((y_global_min - l_b) / l_slope_mean)
        l_x2 = int((y_max - l_b) / l_slope_mean)
        r_x1 = int((y_global_min - r_b) / r_slope_mean)
        r_x2 = int((y_max - r_b) / r_slope_mean)

        # 6
        if l_x1 > r_x1:
            l_x1 = int((l_x1 + r_x1) / 2)
            r_x1 = l_x1
            l_y1 = int((l_slope_mean * l_x1) + l_b)
            r_y1 = int((r_slope_mean * r_x1) + r_b)
            l_y2 = int((l_slope_mean * l_x2) + l_b)
            r_y2 = int((r_slope_mean * r_x2) + r_b)
        else:
            l_y1 = y_global_min
            l_y2 = y_max
            r_y1 = y_global_min
            r_y2 = y_max

        current_frame = np.array([l_x1, l_y1, l_x2, l_y2, r_x1, r_y1, r_x2, r_y2], dtype="float32")

        if self.first_frame == 1:
            next_frame = current_frame
            self.first_frame = 0
        else:
            prev_frame = self.cache
            next_frame = (1 - α) * prev_frame + α * current_frame

        cv2.line(img, (int(next_frame[0]), int(next_frame[1])), (int(next_frame[2]), int(next_frame[3])), color,
                 thickness)
        cv2.line(img, (int(next_frame[4]), int(next_frame[5])), (int(next_frame[6]), int(next_frame[7])), color,
                 thickness)

        cache = next_frame

    def get_slope(self, x1, y1, x2, y2):
        return (y2 - y1) / (x2 - x1)

    def get_inputs_keys(self) -> List[str]:
        return [self._input]

    def get_outputs_keys(self) -> List[str]:
        return [self._output]
