import logging
import typing

import cv2
import numpy as np
import pytest
from paho.mqtt.client import Client
from pytest_docker_compose import NetworkInfo

from donkeycar.parts.camera_pilot import AngleProcessorMiddleLine, \
    ThresholdValueEstimator, ThresholdController, \
    ThresholdConfigController
from donkeycar.tests.conftest import wait_port_open, wait_all_mqtt_messages_consumed

logger = logging.getLogger(__name__)


def _load_img(img):
    path = "donkeycar/tests/data/" + img
    return cv2.imread(path)


def _load_img_gray(img):
    return cv2.cvtColor(_load_img(img), cv2.COLOR_RGB2GRAY)


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


@pytest.fixture
def threshold_controller():
    return ThresholdController()


class TestThresholdController:

    def test_straight_line(self, threshold_controller: ThresholdController):
        assert len(threshold_controller.run(_load_img_gray("straight_line_1.jpg"),
                                            threshold_limit_min=180, threshold_limit_max=255)) > 0

    def test_threshold_min_max(self, threshold_controller: ThresholdController):
        img_gray = np.ones((256, 256))
        for i in range(0, 256):
            img_gray[i] = np.ones(256) * i

        img = threshold_controller.run(img_gray, threshold_limit_min=170, threshold_limit_max=190)

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


0


@pytest.fixture(name='threshold_config_controller_mqtt')
def fixture_threshold_config_controller_mqtt(docker_network_info: typing.Dict[str, typing.List[NetworkInfo]]):
    mqtt_service = docker_network_info["donkeycar_mqtt_1"][0]
    host = 'localhost'
    port = 1883
    wait_port_open(host=host, port=port)
    return ThresholdConfigController(limit_min=150, limit_max=200,
                                     threshold_dynamic=True, threshold_default=160, threshold_delta=20,
                                     mqtt_enable=True,
                                     mqtt_hostname=host,
                                     mqtt_port=port,
                                     mqtt_qos=1,
                                     mqtt_topic='test/car/config/threshold/#')


@pytest.fixture(name='threshold_config_controller_static')
def fixture_threshold_config_controller_static():
    return ThresholdConfigController(limit_min=150, limit_max=200,
                                     threshold_dynamic=False, threshold_default=160, threshold_delta=20,
                                     mqtt_enable=False)


class TestThresholdConfigController:

    def test_static_value(self, threshold_config_controller_static: ThresholdConfigController):
        t_min, t_max, t_dynamic_enabled, t_default, t_delta = threshold_config_controller_static.run(100)
        assert t_min == 150
        assert t_max == 200
        assert t_default == 160

    def test_dynamic_value(self, threshold_config_controller_static: ThresholdConfigController):
        threshold_config_controller_static.dynamic_enabled = True
        t_min, t_max, t_dynamic_enabled, t_default, t_delta = threshold_config_controller_static.run(100)
        assert t_min == 80
        assert t_max == 120
        assert t_default == 100
        assert t_delta == 20

    def test_modify_config_min_max_with_mqtt(self, threshold_config_controller_mqtt: ThresholdConfigController,
                                             mqtt_config: Client):
        threshold_config_controller_mqtt.dynamic_enabled = False
        t_min, t_max, t_dynamic_enabled, t_default, t_delta = threshold_config_controller_mqtt.run(100)
        assert t_min == 150
        assert t_max == 200

        wait_all_mqtt_messages_consumed(f'mqtt-subscription-{threshold_config_controller_mqtt._mqtt_client_id}'
                                        f'qos{threshold_config_controller_mqtt.qos}')

        msg = mqtt_config.publish(topic='test/car/config/threshold/min', payload="200", qos=1)
        msg.wait_for_publish()
        msg = mqtt_config.publish(topic='test/car/config/threshold/max', payload="220", qos=1)
        msg.wait_for_publish()

        wait_all_mqtt_messages_consumed(
            f'mqtt-subscription-{threshold_config_controller_mqtt._mqtt_client_id}'
            f'qos{threshold_config_controller_mqtt.qos}')
        t_min, t_max, t_dynamic_enabled, t_default, t_delta = threshold_config_controller_mqtt.run(100)
        assert t_min == 200
        assert t_max == 220

    def test_modify_config_dynamic_with_mqtt(self, threshold_config_controller_mqtt: ThresholdConfigController,
                                             mqtt_config: Client):
        threshold_config_controller_mqtt.dynamic_enabled = False
        t_min, t_max, t_dynamic_enabled, t_default, t_delta = threshold_config_controller_mqtt.run(100)
        assert t_min == 150
        assert t_max == 200
        assert t_dynamic_enabled == False
        assert t_default == 160
        assert t_delta == 20

        wait_all_mqtt_messages_consumed(f'mqtt-subscription-{threshold_config_controller_mqtt._mqtt_client_id}'
                                        f'qos{threshold_config_controller_mqtt.qos}')

        msg = mqtt_config.publish(topic='test/car/config/threshold/default', payload="200", qos=1)
        msg.wait_for_publish()
        msg = mqtt_config.publish(topic='test/car/config/threshold/delta', payload="5", qos=1)
        msg.wait_for_publish()
        msg = mqtt_config.publish(topic='test/car/config/threshold/dynamic_enabled', payload="true", qos=1)
        msg.wait_for_publish()

        wait_all_mqtt_messages_consumed(
            f'mqtt-subscription-{threshold_config_controller_mqtt._mqtt_client_id}'
            f'qos{threshold_config_controller_mqtt.qos}')
        t_min, t_max, t_dynamic_enabled, t_default, t_delta = threshold_config_controller_mqtt.run(100)
        assert t_min == 95
        assert t_max == 105
        assert t_dynamic_enabled == True
        assert t_default == 100
        assert t_delta == 5
