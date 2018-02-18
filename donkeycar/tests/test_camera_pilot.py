import cv2
import numpy
import pytest

from donkeycar.parts.camera_pilot import ImagePilot, FIXED_THROTTLE, AngleProcessorMiddleLine


@pytest.fixture
def pilot():
    return ImagePilot()


class TestDrive:
    @pytest.mark.skip
    def test_straight_line(self, pilot):
        img = self._load_img('straight_line_1.jpg')
        angle, throttle = pilot.run(img)
        pass

    def test_turn_right(self, pilot):
        img = self._load_img('turn_right.jpg')

        angle, throttle = pilot.run(img)
        assert throttle is FIXED_THROTTLE
        assert 0.9 >= angle >= 0.2

    @staticmethod
    def _load_img(img):
        path = "donkeycar/tests/data/" + img
        return cv2.imread(path)


@pytest.fixture()
def angle_processor():
    return AngleProcessorMiddleLine()


class TestAngleEstimatorMiddleLine:

    def test_no_line(self, angle_processor):
        centroids = []
        assert angle_processor.estimate(centroids, image_size=128) == 0.0

    def test_middle_line_on_border_left(self, angle_processor):
        centroids = [(2, 12)]
        assert angle_processor.estimate(centroids, image_size=128) == -1.0

    def test_middle_line_on_left(self, angle_processor):
        centroids = [(33, 0)]
        angle = angle_processor.estimate(centroids, image_size=128)
        assert -0.9 < angle < -0.2

    def test_middle_line_on_middle(self, angle_processor):
        centroids = [(65, 0)]
        angle = angle_processor.estimate(centroids, image_size=128)
        assert angle == 0.0

        centroids = [(63, 0)]
        angle = angle_processor.estimate(centroids, image_size=128)
        assert angle == 0.0

    def test_middle_line_on_right(self, angle_processor):
        centroids = [(90, 0)]
        angle = angle_processor.estimate(centroids, image_size=128)
        assert 0.2 < angle < 0.9

    def test_middle_line_on_border_right(self, angle_processor):
        centroids = [(120, 0)]
        angle = angle_processor.estimate(centroids, image_size=128)
        assert angle == 1.0
