import logging

import pytest
from paho.mqtt.client import Client

from donkeycar.parts.angle import AngleProcessorMiddleLine, AngleConfigController
from donkeycar.tests.conftest import wait_port_open, wait_all_mqtt_messages_consumed, NetworkInfo

logger = logging.getLogger(__name__)


class TestAngleEstimatorMiddleLine:
    @pytest.fixture(name='angle_processor')
    def fixture_angle_processor(self):
        return AngleProcessorMiddleLine(image_resolution=(120, 160))

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

    def test_2_centroids_on_left(self, angle_processor: AngleProcessorMiddleLine):
        centroids = [(30, 0), (40, 20)]
        angle = angle_processor.run(centroids)
        assert angle == -0.625

        centroids.reverse()
        angle = angle_processor.run(centroids)
        assert angle == -0.5

    def test_2_centroids_on_right(self, angle_processor: AngleProcessorMiddleLine):
        centroids = [(90, 0), (100, 20)]
        angle = angle_processor.run(centroids)
        assert angle == 0.125

        centroids.reverse()
        angle = angle_processor.run(centroids)
        assert angle == 0.25

    def test_2_centroids_on_left_and_right(self, angle_processor: AngleProcessorMiddleLine):
        centroids = [(60, 0), (100, 20)]
        angle = angle_processor.run(centroids)
        assert angle == -0.25

        centroids.reverse()
        angle = angle_processor.run(centroids)
        assert angle == 0.25


class TestAngleConfigController:
    @pytest.fixture(name='angle_config_controller')
    def fixture_angle_config_controller_mqtt(self, mqtt_service: NetworkInfo) -> AngleConfigController:
        host = 'localhost'
        port = 1883
        wait_port_open(host=host, port=port)

        angle_config = AngleConfigController(out_zone_percent=10, central_zone_percent=20, mqtt_enable=True,
                                             mqtt_topic='test/car/config/angle/#', mqtt_hostname=host, mqtt_port=port,
                                             mqtt_client_id='donkey-config-angle-', mqtt_qos=1)

        wait_all_mqtt_messages_consumed(f'mqtt-subscription-{angle_config._mqtt_client_id}'
                                        f'qos{angle_config.qos}')
        yield angle_config
        angle_config.shutdown()

    def test_values(self, angle_config_controller: AngleConfigController, mqtt_config: Client):
        number_centroids_to_use, out_zone_percent, central_zone_percent = angle_config_controller.run()
        assert number_centroids_to_use == 1
        assert out_zone_percent == 10
        assert central_zone_percent == 20

        mqtt_config.publish(topic='test/car/config/angle/number_centroids_to_use', payload="5", qos=1) \
            .wait_for_publish()
        mqtt_config.publish(topic='test/car/config/angle/out_zone_percent', payload="50", qos=1) \
            .wait_for_publish()
        mqtt_config.publish(topic='test/car/config/angle/central_zone_percent', payload="60", qos=1) \
            .wait_for_publish()

        wait_all_mqtt_messages_consumed(
            f'mqtt-subscription-{angle_config_controller._mqtt_client_id}'
            f'qos{angle_config_controller.qos}')
        number_centroids_to_use, out_zone_percent, central_zone_percent = angle_config_controller.run()
        assert number_centroids_to_use == 5
        assert out_zone_percent == 50
        assert central_zone_percent == 60
