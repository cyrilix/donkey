import logging
import os
from pathlib import Path
from typing import List

import numpy as np

from donkeycar.parts.angle import PILOT_ANGLE
from donkeycar.parts.part import Part

try:
    import keras
except:
    logging.warning('Unable to import keras')
import donkeycar as dk


class KerasPilot(Part):

    def __init__(self, img_input: str, model_path: Path):
        self._img_input = img_input
        self.model = keras.models.load_model(str(model_path))

    def run(self, img_arr):
        img_arr = img_arr.reshape((1,) + img_arr.shape)
        angle_binned, throttle = self.model.predict(img_arr)
        print(angle_binned)
        #print('throttle', throttle)
        #angle_certainty = max(angle_binned[0])
        angle_unbinned = dk.utils.linear_unbin(angle_binned)
        return angle_unbinned

    def get_inputs_keys(self) -> List[str]:
        return [self._img_input]

    def get_outputs_keys(self) -> List[str]:
        return [PILOT_ANGLE]
