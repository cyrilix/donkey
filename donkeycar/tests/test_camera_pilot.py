import cv2
import pytest

from donkeycar.parts.camera_pilot import ImagePilot, FIXED_THROTTLE, AngleProcessorMiddleLine


@pytest.fixture
def pilot():
    return ImagePilot(AngleProcessorMiddleLine(image_resolution=(120, 160)))


class TestDrive:

    def test_straight_line(self, pilot):
        img = self._load_img('straight_line_1.jpg')
        angle, throttle = pilot.run(img)

        assert throttle is FIXED_THROTTLE
        assert 0.1 >= angle

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
