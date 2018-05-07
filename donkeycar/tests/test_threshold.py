import numpy as np
import numpy as np
import pytest
from paho.mqtt.client import Client

from donkeycar.parts.threshold import ThresholdController, ThresholdValueEstimator, ThresholdConfigController, \
    ContoursConfigController, ContoursDetector, ThresholdValueEstimatorConfig
from donkeycar.tests.conftest import wait_port_open, wait_all_mqtt_messages_consumed, NetworkInfo


@pytest.fixture(name='threshold_config_controller_static')
def fixture_threshold_config_controller_static() -> ThresholdConfigController:
    return ThresholdConfigController(limit_min=150, limit_max=200,
                                     threshold_dynamic=False, threshold_default=160, threshold_delta=20,
                                     mqtt_enable=False)


@pytest.fixture
def threshold_controller(threshold_config_controller_static) -> ThresholdController:
    return ThresholdController(config=threshold_config_controller_static)


class TestThresholdController:

    def test_straight_line(self, threshold_controller: ThresholdController, img_straight_line_gray: np.ndarray):
        assert len(threshold_controller.run(image_gray=img_straight_line_gray)) > 0

    def test_threshold_min_max(self, threshold_controller: ThresholdController):
        threshold_controller._config.limit_min = 170
        threshold_controller._config.limit_max = 190

        img_gray = np.ones((256, 256))
        for i in range(0, 256):
            img_gray[i] = np.ones(256) * i

        img = threshold_controller.run(img_gray)

        for i in range(170):
            assert list(img[(i, ...)]) == list(np.zeros((256,)))

        for i in range(171, 190):
            assert list(img[i]) == list(np.ones((256,)) * 255)

        for i in range(191, 256):
            assert list(img[i]) == list(np.zeros((256,)))


class TestThresholdValueEstimatorConfigController:
    @pytest.fixture(name='threshold_value_config')
    def fixture_threshold_value_config(self, mqtt_service: NetworkInfo) -> ThresholdValueEstimatorConfig:
        host = 'localhost'
        port = 1883
        wait_port_open(host=host, port=port)
        return ThresholdValueEstimatorConfig(centroid_value=120,
                                             mqtt_enable=True,
                                             mqtt_hostname=host,
                                             mqtt_port=port,
                                             mqtt_qos=1,
                                             mqtt_client_id='donkey-config-threshold_value-',
                                             mqtt_topic='test/car/config/threshold_value_estimator/#')

    def test_run(self, threshold_value_config: ThresholdValueEstimatorConfig, mqtt_config: Client):
        centroid_value = threshold_value_config.run()
        assert centroid_value == 120

        wait_all_mqtt_messages_consumed(f'mqtt-subscription-{threshold_value_config._mqtt_client_id}'
                                        f'qos{threshold_value_config.qos}')
        mqtt_config.publish(topic='test/car/config/threshold_value_estimator/centroid_value',
                            payload="220", qos=1).wait_for_publish()
        wait_all_mqtt_messages_consumed(f'mqtt-subscription-{threshold_value_config._mqtt_client_id}'
                                        f'qos{threshold_value_config.qos}')

        centroid_value = threshold_value_config.run()
        assert centroid_value == 220


class TestThresholdValueEstimator:

    def test_get_value(self, img_straight_line_gray: np.ndarray):
        value_estimator = ThresholdValueEstimator(contours_detector=190)

        assert value_estimator.run(img_gray=img_straight_line_gray) == 190


@pytest.fixture(name='threshold_config_controller_mqtt')
def fixture_threshold_config_controller_mqtt(mqtt_service: NetworkInfo) \
        -> ThresholdConfigController:
    host = 'localhost'
    port = 1883
    wait_port_open(host=host, port=port)
    return ThresholdConfigController(limit_min=150, limit_max=200,
                                     threshold_dynamic=True, threshold_default=160, threshold_delta=20,
                                     mqtt_enable=True,
                                     mqtt_hostname=host,
                                     mqtt_port=port,
                                     mqtt_qos=1,
                                     mqtt_client_id='donkey-config-threshold-',
                                     mqtt_topic='test/car/config/threshold/#')


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


@pytest.fixture(name='config_contours')
def fixture_contours_config_controller(mqtt_service: NetworkInfo) -> ContoursConfigController:
    host = 'localhost'
    port = 1883
    wait_port_open(host=host, port=port)
    return ContoursConfigController(poly_dp_min=4, poly_dp_max=100,
                                    arc_length_min=10, arc_length_max=100000,
                                    mqtt_enable=True,
                                    mqtt_hostname=host,
                                    mqtt_port=port,
                                    mqtt_qos=1,
                                    mqtt_client_id='donkey-config-contours-',
                                    mqtt_topic='test/car/config/contours/#')


class TestContoursConfigController:

    def test_config(self, config_contours: ContoursConfigController, mqtt_config: Client):
        poly_dp_min, poly_dp_max, arc_length_min, arc_length_max = config_contours.run()

        assert arc_length_min == 10
        assert arc_length_max == 100000
        assert poly_dp_min == 4
        assert poly_dp_max == 100

        wait_all_mqtt_messages_consumed(
            f'mqtt-subscription-{config_contours._mqtt_client_id}'
            f'qos{config_contours.qos}')

        mqtt_config.publish(topic='test/car/config/contours/arc_length_min', payload="1", qos=1).wait_for_publish()
        mqtt_config.publish(topic='test/car/config/contours/arc_length_max', payload="10", qos=1).wait_for_publish()
        mqtt_config.publish(topic='test/car/config/contours/poly_dp_min', payload="5", qos=1).wait_for_publish()
        mqtt_config.publish(topic='test/car/config/contours/poly_dp_max', payload="20", qos=1).wait_for_publish()

        wait_all_mqtt_messages_consumed(
            f'mqtt-subscription-{config_contours._mqtt_client_id}'
            f'qos{config_contours.qos}')

        poly_dp_min, poly_dp_max, arc_length_min, arc_length_max = config_contours.run()

        assert arc_length_min == 1
        assert arc_length_max == 10
        assert poly_dp_min == 5
        assert poly_dp_max == 20


@pytest.fixture(name='contours_detector')
def fixture_contours_detector() -> ContoursDetector:
    return ContoursDetector(config=ContoursConfigController(mqtt_enable=False))


class TestContoursDetector:

    def test_process(self, contours_detector: ContoursDetector, img_straight_line_binarized_150_200: np.ndarray):
        contours, centroids = contours_detector.process_image(img_binarized=img_straight_line_binarized_150_200)
        assert len(contours) == 5
        assert len(centroids) == 5

        contours_detector._config.poly_dp_max = 6
        contours, centroids = contours_detector.process_image(img_binarized=img_straight_line_binarized_150_200)
        assert len(contours) == 4
        assert len(centroids) == 4
