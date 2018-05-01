import logging

import pytest

from donkeycar.parts.angle import AngleProcessorMiddleLine

logger = logging.getLogger(__name__)


@pytest.fixture()
def angle_processor():
    return AngleProcessorMiddleLine(image_resolution=(120, 160))


class TestAngleEstimatorMiddleLine:

    def test_no_line(self, angle_processor: AngleProcessorMiddleLine):
        centroids = []
        assert angle_processor.run(centroids) == 0.0

    def test_middle_line_on_border_left(self, angle_processor: AngleProcessorMiddleLine):
        centroids = [(2, 12)]
        assert angle_processor.run(centroids) == -1.0

    def test_middle_line_on_left(self, angle_processor: AngleProcessorMiddleLine):
        centroids = [(33, 0)]
        angle = angle_processor.run(centroids)
        assert -0.9 < angle < -0.2

    def test_middle_line_on_middle(self, angle_processor: AngleProcessorMiddleLine):
        centroids = [(75, 0)]
        angle = angle_processor.run(centroids)
        assert angle == 0.0

        centroids = [(85, 0)]
        angle = angle_processor.run(centroids)
        assert angle == 0.0

    def test_middle_line_on_right(self, angle_processor: AngleProcessorMiddleLine):
        centroids = [(120, 0)]
        angle = angle_processor.run(centroids)
        assert 0.2 < angle < 0.9

    def test_middle_line_on_border_right(self, angle_processor: AngleProcessorMiddleLine):
        centroids = [(155, 0)]
        angle = angle_processor.run(centroids)
        assert angle == 1.0
