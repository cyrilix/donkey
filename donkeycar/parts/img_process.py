import logging
from typing import List

import cv2
import numpy
from numpy.core.multiarray import ndarray

from donkeycar.parts.camera import CAM_IMAGE
from donkeycar.parts.part import Part

IMG_GRAY = 'img/gray'
IMG_GRAY_RAW = 'img/gray/raw'
IMG_GRAY_EQUALIZED = 'img/gray/equalized'


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
        return [IMG_GRAY_RAW]


class GraySelectorPart(Part):
    def run(self, img_gray_raw, img_gray_equalized):
        return img_gray_equalized

    def get_inputs_keys(self) -> List[str]:
        return [IMG_GRAY_RAW, IMG_GRAY_EQUALIZED]

    def get_outputs_keys(self) -> List[str]:
        return [IMG_GRAY]
