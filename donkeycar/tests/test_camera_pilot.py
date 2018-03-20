import logging

import cv2
import numpy as np
import pytest

from donkeycar.parts.camera_pilot import ImagePilot, AngleProcessorMiddleLine, \
    ThrottleControllerFixedSpeed, ThresholdController, ThresholdValueEstimator, ThrottleControllerSteeringBased

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
    return ThresholdController(threshold_defaut=100, threshold_delta=10)


class TestThresholdController:

    def test_straight_line(self, threshold_controller):
        assert len(threshold_controller.run(_load_img_gray("straight_line_1.jpg"), threshold_value=180)) > 0

    def test_threshold_min_max(self, threshold_controller):
        img_gray = np.ones((256, 256))
        for i in range(0, 256):
            img_gray[i] = np.ones(256) * i

        img = threshold_controller.run(img_gray, threshold_value=180)

        for i in range(170):
            assert list(img[(i, ...)]) == list(np.zeros((256,)))

        for i in range(171, 190):
            assert list(img[i]) == list(np.ones((256,)) * 255)

        for i in range(191, 256):
            assert list(img[i]) == list(np.zeros((256,)))


class TestThresholdValueEstimator:

    def test_get_value(self):
        img = _load_img_gray('straight_line_1.jpg')
        value_estimator = ThresholdValueEstimator(init_value=200)

        assert not value_estimator.video_frame()
        assert value_estimator.run(img_gray=img) == 217


@pytest.fixture
def throttle_controller_angle():
    return ThrottleControllerSteeringBased(min_speed=0.4, max_speed=0.8, safe_angle=0.2, dangerous_angle=0.8)


class TestThrottleControllerSteeringBased:

    def test_throttle_with_min_angle(self, throttle_controller_angle):
        assert throttle_controller_angle.run(0.0) == 0.8

    def test_throttle_with_max_angle(self, throttle_controller_angle):
        assert throttle_controller_angle.run(1.0) == 0.4

    def test_throttle_with_intermediate_angle(self, throttle_controller_angle):
        assert throttle_controller_angle.run(0.5) == 0.6
        assert throttle_controller_angle.run(0.8) == 0.72
        assert throttle_controller_angle.run(0.2) == 0.48

