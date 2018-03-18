import logging

import cv2
import numpy as np
import pytest

from donkeycar.parts.camera_pilot import ImagePilot, AngleProcessorMiddleLine, \
    ThrottleControllerFixedSpeed, ThresholdController, ThresholdValueEstimator

logger = logging.getLogger(__name__)


@pytest.fixture
def pilot():
    return ImagePilot(angle_estimator=AngleProcessorMiddleLine(image_resolution=(120, 160)),
                      throttle_controller=ThrottleControllerFixedSpeed(throttle_value=0.1))


def _load_img(img):
    path = "donkeycar/tests/data/" + img
    return cv2.imread(path)


def _load_img_gray(img):
    return cv2.cvtColor(_load_img(img), cv2.COLOR_RGB2GRAY)


class TestDrive:

    def test_blank(self, pilot):
        assert pilot.run([]) == (0.0, 0.1)
        assert pilot.run(None) == (0.0, 0.1)

    def test_on_error(self, pilot):
        def throw_error(centroids):
            raise ValueError()

        pilot._angle_estimator.__dict__['estimate'] = throw_error
        assert pilot.run([(10, 20)]) == (0.0, 0.0)

    def test_straight_line(self, pilot):
        angle, throttle = pilot.run([(75, 110), (80, 85)])

        assert throttle == 0.1
        assert 0.1 >= angle

    def test_turn_right(self, pilot):
        angle, throttle = pilot.run([(150, 150), (150, 140)])
        assert throttle == 0.1
        assert 0.9 >= angle >= 0.2


@pytest.fixture()
def angle_processor():
    return AngleProcessorMiddleLine(image_resolution=(120, 160))


class TestAngleEstimatorMiddleLine:

    def test_no_line(self, angle_processor):
        centroids = []
        assert angle_processor.estimate(centroids) == 0.0

    def test_middle_line_on_border_left(self, angle_processor):
        centroids = [(2, 12)]
        assert angle_processor.estimate(centroids) == -1.0

    def test_middle_line_on_left(self, angle_processor):
        centroids = [(33, 0)]
        angle = angle_processor.estimate(centroids)
        assert -0.9 < angle < -0.2

    def test_middle_line_on_middle(self, angle_processor):
        centroids = [(75, 0)]
        angle = angle_processor.estimate(centroids)
        assert angle == 0.0

        centroids = [(85, 0)]
        angle = angle_processor.estimate(centroids)
        assert angle == 0.0

    def test_middle_line_on_right(self, angle_processor):
        centroids = [(120, 0)]
        angle = angle_processor.estimate(centroids)
        assert 0.2 < angle < 0.9

    def test_middle_line_on_border_right(self, angle_processor):
        centroids = [(155, 0)]
        angle = angle_processor.estimate(centroids)
        assert angle == 1.0


class TestThrottleControllerFixedSpeed:

    def test_run(self):
        assert ThrottleControllerFixedSpeed(throttle_value=0.1).run([]) == 0.1
        assert ThrottleControllerFixedSpeed(throttle_value=0.8).run([]) == 0.8


@pytest.fixture
def threshold_controller():
    return ThresholdController(limit_min=100, limit_max=120)


class TestThresholdController:

    def test_straight_line(self, threshold_controller):
        assert len(threshold_controller.run(_load_img_gray("straight_line_1.jpg"))) > 0

    def test_threshold_min_max(self, threshold_controller):
        img_gray = np.ones((256, 256))
        for i in range(0, 256):
            img_gray[i] = np.ones(256) * i

        img = threshold_controller.run(img_gray)

        for i in range(99):
            assert list(img[(i, ...)]) == list(np.zeros((256,)))

        for i in range(110, 121):
            assert list(img[i]) == list(np.ones((256,)) * 255)

        for i in range(121, 256):
            assert list(img[i]) == list(np.zeros((256,)))


class TestThresholdValueEstimator:

    def test_get_value(self):
        img = _load_img_gray('straight_line_1.jpg')
        value_estimator = ThresholdValueEstimator(init_value=200)

        assert not value_estimator.cache_value()
        assert value_estimator.run(img_gray=img) == 217
