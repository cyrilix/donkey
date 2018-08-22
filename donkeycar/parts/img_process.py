import logging
from typing import List

import cv2
import numpy as np
from numpy.core.multiarray import ndarray

from donkeycar.parts.camera import CAM_IMAGE
from donkeycar.parts.part import Part

IMG_GRAY = 'img/gray'
IMG_GRAY_RAW = 'img/gray/raw'
IMG_GRAY_EQUALIZED = 'img/gray/equalized'

logger = logging.getLogger(__name__)


class HistogramPart(Part):
    def run(self, img_gray: ndarray) -> ndarray:
        try:
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            return clahe.apply(img_gray.copy())
        except Exception:
            logging.exception("Unexpected error")
            return None

    def get_inputs_keys(self) -> List[str]:
        return [IMG_GRAY_RAW]

    def get_outputs_keys(self) -> List[str]:
        return [IMG_GRAY_EQUALIZED]


class BlurPart(Part):
    def __init__(self, input_key: str, output_key: str):
        self._input_keys = [input_key]
        self._output_keys = [output_key]

    def run(self, img: ndarray) -> ndarray:
        try:
            return cv2.bilateralFilter(img.copy(), d=5, sigmaSpace=75, sigmaColor=75)
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

    def run(self, img: ndarray, road_contour) -> ndarray:
        try:
            if not road_contour:
                logger.info('no road')
                return img
            logger.info(road_contour)
            x, y, w, h = cv2.boundingRect(np.array(road_contour))
            if w < 20 or h < 20:
                return img
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

    def run(self, image_array: ndarray) -> ndarray:
        try:
            return cv2.cvtColor(image_array.copy(), cv2.COLOR_RGB2GRAY)
        except Exception:
            logging.exception("Unexpected error")
            return np.zeros(image_array.shape)

    def get_inputs_keys(self) -> List[str]:
        return [CAM_IMAGE]

    def get_outputs_keys(self) -> List[str]:
        return [IMG_GRAY_RAW]


class GraySelectorPart(Part):
    def run(self, img_gray_raw, img_gray_equalized):
        return img_gray_equalized

    def get_inputs_keys(self) -> List[str]:
        return [IMG_GRAY_RAW, IMG_GRAY_EQUALIZED]

    def get_outputs_keys(self) -> List[str]:
        return [IMG_GRAY]
