from typing import Iterator, Tuple

import cv2
import numpy
import pytest as pytest
from numpy import ndarray
from paho.mqtt.client import Client

from donkeycar.parts.road import RoadPart, RoadConfigController, CFG_ROAD_HORIZON_HOUGH_MIN_LINE_LENGTH, \
    CFG_ROAD_HORIZON_HOUGH_MAX_LINE_GAP, CFG_ROAD_HORIZON_HOUGH_THRESHOLD, CFG_ROAD_CONTOUR_KERNEL_SIZE, \
    CFG_ROAD_CONTOUR_MORPHO_ITERATIONS, CFG_ROAD_CONTOUR_CANNY_THRESHOLD1, CFG_ROAD_CONTOUR_CANNY_THRESHOLD2, \
    CFG_ROAD_CONTOUR_APPROX_POLY_EPSILON_FACTOR, CFG_ROAD_ENABLE, ComponentRoadPart, RoadEllipsePart
from donkeycar.parts.threshold import Shape
from donkeycar.tests.conftest import wait_port_open, wait_all_mqtt_messages_consumed
from pytest import fixture


class TestComponentRoadPart:
    @fixture(name='component_road_part')
    def fixture_component_road_part(self) -> ComponentRoadPart:
        return ComponentRoadPart()

    def test_run(self, component_road_part: ComponentRoadPart, img_straight_line: ndarray):
        _, _, _, _, contour, _, img_debug, ellipse = component_road_part.run(img_straight_line)

        # Uncomment to debug
        # self.debug_contour(img_debug.copy(), contour)

        expected_contour = [(0, 31),
                            (0, 119),
                            (43, 119),
                            (45, 110),
                            (46, 119),
                            (66, 119),
                            (67, 100),
                            (68, 119),
                            (159, 119),
                            (158, 42),
                            (87, 18),
                            (73, 22),
                            (71, 53),
                            (61, 47),
                            (64, 20)]
        assert contour == expected_contour
        assert len(img_debug) > 0
        assert ellipse.center == (73, 72)
        assert 96 < ellipse.axes[0] < 97
        assert 299 < ellipse.axes[1] < 300
        assert 87 < ellipse.angle < 93
        assert ellipse.trust == 1.0

    def test_invalid_image(self, component_road_part: ComponentRoadPart, img_black: ndarray):
        _, _, _, _, contour, _, img_debug, ellipse = component_road_part.run(img_black)
        assert ellipse.trust == 0.0
        assert not ellipse.center
        assert not ellipse.axes
        assert ellipse.angle == 90

    @staticmethod
    def debug_contour(img_debug: ndarray, contour: Shape) -> None:
        img_debug = cv2.polylines(img_debug, pts=[numpy.array(contour)],
                                  isClosed=True,
                                  color=(0, 255, 0))
        cv2.imwrite(filename='/tmp/debug.jpg', img=img_debug)


@pytest.mark.skip()
class TestRoad:

    @pytest.fixture(name='road_part')
    def fixture_road_part(self) -> RoadPart:
        road_config = RoadConfigController(enable=True,
                                           canny_threshold1=180,
                                           canny_threshold2=200,
                                           kernel_size=4,
                                           mqtt_enable=False)
        return RoadPart(config=road_config)

    def test_image(self, road_part: RoadPart, img_straight_line_gray: ndarray) -> None:
        contour, horizon = road_part.run(img_gray=img_straight_line_gray)

        # Uncomment to debug
        self.debug_contour(img_straight_line_gray.copy(), contour)

        expected_contour = [(0, 30),
                            (0, 119),
                            (43, 119),
                            (45, 112),
                            (46, 119),
                            (68, 119),
                            (69, 96),
                            (70, 119),
                            (159, 119),
                            (159, 41),
                            (81, 14),
                            (105, 11),
                            (51, 13)]

        assert len(contour) > 4
        assert contour == expected_contour

    def test_image_disabled(self, road_part: RoadPart, img_straight_line_gray: ndarray) -> None:
        contour, horizon = road_part.run(img_gray=img_straight_line_gray)
        assert len(contour) > 4

        road_part._config.enable = False
        contour, horizon = road_part.run(img_gray=img_straight_line_gray)
        assert len(contour) == 0

    @staticmethod
    def debug_contour(img_debug: ndarray, contour: Shape) -> None:
        img_debug = cv2.polylines(cv2.cvtColor(img_debug, cv2.COLOR_GRAY2RGB), pts=[numpy.array(contour)],
                                  isClosed=True,
                                  color=(0, 255, 0))
        cv2.imwrite(filename='/tmp/debug.jpg', img=img_debug)

    def test_image_blank(self, road_part: RoadPart) -> None:
        contour, horizon = road_part.run(img_gray=numpy.zeros((100, 100), dtype=numpy.uint8))

        expected_contour = [(0, 1),
                            (0, 99),
                            (99, 99),
                            (99, 1)]

        assert contour == expected_contour


class TestRoadConfigController:
    @pytest.fixture(name='config')
    def road_config_controller(self, mqtt_address: Tuple[str, int]) -> Iterator[RoadConfigController]:
        host = mqtt_address[0]
        port = mqtt_address[1]
        wait_port_open(host=host, port=port)

        road_config = RoadConfigController(mqtt_enable=True,
                                           enable=True,
                                           horizon_hough_max_line_gap=10,
                                           horizon_hough_min_line_length=1,
                                           horizon_hough_threshold=100,
                                           kernel_size=4,
                                           morpho_iterations=3,
                                           canny_threshold1=120,
                                           canny_threshold2=250,
                                           approx_poly_epsilon_factor=0.01,
                                           mqtt_topic='test/car/config/road/#', mqtt_hostname=host, mqtt_port=port,
                                           mqtt_client_id='donkey-config-road-', mqtt_qos=1)

        wait_all_mqtt_messages_consumed(f'mqtt-subscription-{road_config._mqtt_client_id}'
                                        f'qos{road_config.qos}')
        yield road_config
        road_config.shutdown()

    def test_values(self, config: RoadConfigController, mqtt_config: Client):
        enable, horizon_hough_min_line_length, horizon_hough_max_line_gap, horizon_hough_threshold, kernel_size, \
        morpho_iterations, canny_threshold1, canny_threshold2, approx_poly_epsilon_factor = config.run()

        assert enable == True
        assert horizon_hough_min_line_length == 1
        assert horizon_hough_max_line_gap == 10
        assert horizon_hough_threshold == 100
        assert kernel_size == 4
        assert morpho_iterations == 3
        assert canny_threshold1 == 120
        assert canny_threshold2 == 250
        assert approx_poly_epsilon_factor == 0.01

        mqtt_config.publish(topic=f'test/car/config/{CFG_ROAD_ENABLE.replace("cfg/", "")}',
                            payload="false", qos=1).wait_for_publish()
        mqtt_config.publish(topic=f'test/car/config/{CFG_ROAD_HORIZON_HOUGH_MIN_LINE_LENGTH.replace("cfg/", "")}',
                            payload="10", qos=1).wait_for_publish()
        mqtt_config.publish(topic=f'test/car/config/{CFG_ROAD_HORIZON_HOUGH_MAX_LINE_GAP.replace("cfg/", "")}',
                            payload="5", qos=1).wait_for_publish()
        mqtt_config.publish(topic=f'test/car/config/{CFG_ROAD_HORIZON_HOUGH_THRESHOLD.replace("cfg/", "")}',
                            payload="20", qos=1).wait_for_publish()
        mqtt_config.publish(topic=f'test/car/config/{CFG_ROAD_CONTOUR_KERNEL_SIZE.replace("cfg/", "")}',
                            payload="30", qos=1).wait_for_publish()
        mqtt_config.publish(topic=f'test/car/config/{CFG_ROAD_CONTOUR_MORPHO_ITERATIONS.replace("cfg/", "")}',
                            payload="40", qos=1).wait_for_publish()
        mqtt_config.publish(topic=f'test/car/config/{CFG_ROAD_CONTOUR_CANNY_THRESHOLD1.replace("cfg/", "")}',
                            payload="50", qos=1).wait_for_publish()
        mqtt_config.publish(topic=f'test/car/config/{CFG_ROAD_CONTOUR_CANNY_THRESHOLD2.replace("cfg/", "")}',
                            payload="60", qos=1).wait_for_publish()
        mqtt_config.publish(topic=f'test/car/config/{CFG_ROAD_CONTOUR_APPROX_POLY_EPSILON_FACTOR.replace("cfg/", "")}',
                            payload="70.5", qos=1).wait_for_publish()

        wait_all_mqtt_messages_consumed(
            f'mqtt-subscription-{config._mqtt_client_id}'
            f'qos{config.qos}')

        enable, horizon_hough_min_line_length, horizon_hough_max_line_gap, horizon_hough_threshold, kernel_size, \
        morpho_iterations, canny_threshold1, canny_threshold2, approx_poly_epsilon_factor = config.run()

        assert enable == False
        assert horizon_hough_min_line_length == 10
        assert horizon_hough_max_line_gap == 5
        assert horizon_hough_threshold == 20
        assert kernel_size == 30
        assert morpho_iterations == 40
        assert canny_threshold1 == 50
        assert canny_threshold2 == 60
        assert approx_poly_epsilon_factor == 70.5


class TestRoadEllipsePart:
    @fixture(name='instance')
    def fixture_instance(self) -> RoadEllipsePart:
        return RoadEllipsePart()

    def test_run_short_contour(self, instance: RoadEllipsePart):
        ellipse = instance.run(contour=[(0, 0), (10, 0), (10, 20)])

        assert ellipse.trust == 0.0
        assert not ellipse.center
        assert not ellipse.axes
        assert ellipse.angle == 90

    def test_run(self, instance: RoadEllipsePart, monkeypatch):
        contour = [(0, 30), (0, 119), (43, 119), (45, 112), (46, 119), (68, 119), (69, 96), (70, 119), (159, 119),
                   (159, 41), (81, 14), (105, 11), (51, 13)]

        monkeypatch.setattr(cv2, 'fitEllipse', lambda ctr: ((75, 76), (20, 60), 93.0))
        ellipse = instance.run(contour=contour)

        assert ellipse.trust == 1.0
        assert ellipse.center == (75, 76)
        assert ellipse.axes == (20, 60)
        assert ellipse.angle == 93.0

