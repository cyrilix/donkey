from typing import Dict, List

import pytest
from paho.mqtt.client import Client
from pytest_docker_compose import NetworkInfo

from donkeycar.parts.throttle import ThrottleControllerSteeringBased, ThrottleControllerFixedSpeed, \
    ThrottleConfigController
from donkeycar.tests.conftest import wait_port_open, wait_all_mqtt_messages_consumed


class TestThrottleControllerFixedSpeed:

    def test_run(self):
        throttle_controller = ThrottleControllerFixedSpeed()
        assert throttle_controller.run(throttle_value=0.1) == 0.1
        assert throttle_controller.run(throttle_value=0.8) == 0.8

    def test_throttle_with_shock(self):
        controller_fixed_speed = ThrottleControllerFixedSpeed()
        assert controller_fixed_speed.run(throttle_value=0.1, stop_on_shock=True, shock=False) == 0.1
        assert controller_fixed_speed.run(throttle_value=0.1, stop_on_shock=True, shock=True) == 0.0
        assert controller_fixed_speed.run(throttle_value=0.1, stop_on_shock=True, shock=False) == 0.0


@pytest.fixture(name='throttle_controller_angle')
def fixture_throttle_controller_angle():
    return ThrottleControllerSteeringBased()


class TestThrottleControllerSteeringBased:

    def test_throttle_with_min_angle(self, throttle_controller_angle: ThrottleControllerSteeringBased):
        assert throttle_controller_angle.run(angle=0.0, min_speed=0.4, max_speed=0.8, safe_angle=0.1,
                                             dangerous_angle=0.8, stop_on_shock=False, shock=False) == 0.8

    def test_throttle_with_max_angle(self, throttle_controller_angle: ThrottleControllerSteeringBased):
        assert throttle_controller_angle.run(angle=1.0, min_speed=0.4, max_speed=0.8, safe_angle=0.1,
                                             dangerous_angle=0.8, stop_on_shock=False, shock=False) == 0.4

    def test_throttle_with_intermediate_angle(self, throttle_controller_angle: ThrottleControllerSteeringBased):
        assert throttle_controller_angle.run(angle=0.5, min_speed=0.4, max_speed=0.8, safe_angle=0.1,
                                             dangerous_angle=0.8, stop_on_shock=False, shock=False) == 0.6
        assert throttle_controller_angle.run(angle=0.8, min_speed=0.4, max_speed=0.8, safe_angle=0.1,
                                             dangerous_angle=0.8, stop_on_shock=False, shock=False) == 0.72
        assert throttle_controller_angle.run(angle=0.2, min_speed=0.4, max_speed=0.8, safe_angle=0.2,
                                             dangerous_angle=0.8, stop_on_shock=False, shock=False) == 0.48

    def test_throttle_with_shock(self, throttle_controller_angle: ThrottleControllerSteeringBased):
        assert throttle_controller_angle.run(angle=1.0, min_speed=0.4, max_speed=0.8, safe_angle=0.1,
                                             dangerous_angle=0.8, stop_on_shock=True, shock=False) > 0.1
        assert throttle_controller_angle.run(angle=1.0, min_speed=0.4, max_speed=0.8, safe_angle=0.1,
                                             dangerous_angle=0.8, stop_on_shock=True, shock=True) == 0.0
        assert throttle_controller_angle.run(angle=1.0, min_speed=0.4, max_speed=0.8, safe_angle=0.1,
                                             dangerous_angle=0.8, stop_on_shock=True, shock=False) == 0.


@pytest.fixture(name='throttle_config_controller')
def fixture_throttle_config_controller():
    return ThrottleConfigController(min_speed=0.1, max_speed=1, safe_angle=0.2, dangerous_angle=0.8,
                                    stop_on_shock=True, use_steering=False, mqtt_enable=False)


@pytest.fixture(name='throttle_config_controller_mqtt')
def fixture_throttle_config_controller_mqtt(docker_network_info: Dict[str, List[NetworkInfo]]):
    mqtt_service = docker_network_info["donkeycar_mqtt_1"][0]
    host = 'localhost'
    port = 1883
    wait_port_open(host=host, port=port)

    throttle_config = ThrottleConfigController(min_speed=0.1, max_speed=1, safe_angle=0.2, dangerous_angle=0.8,
                                               stop_on_shock=True, use_steering=False,
                                               mqtt_enable=True,
                                               mqtt_hostname=host,
                                               mqtt_port=port,
                                               mqtt_qos=1,
                                               mqtt_topic='test/car/config/throttle/#')

    wait_all_mqtt_messages_consumed(f'mqtt-subscription-{throttle_config._mqtt_client_id}'
                                    f'qos{throttle_config.qos}')
    return throttle_config


class TestThrottleConfigController:

    def test_values(self, throttle_config_controller: ThrottleConfigController):
        use_steering, min_speed, max_speed, safe_angle, dangerous_angle, stop_on_shock = throttle_config_controller.run()
        assert use_steering == False
        assert min_speed == 0.1
        assert max_speed == 1.0
        assert safe_angle == 0.2
        assert dangerous_angle == 0.8
        assert stop_on_shock == True

    def test_modify_config_with_mqtt(self, throttle_config_controller_mqtt: ThrottleConfigController,
                                     mqtt_config: Client):
        use_steering, min_speed, max_speed, safe_angle, dangerous_angle, stop_on_shock = throttle_config_controller_mqtt.run()
        assert use_steering == False
        assert min_speed == 0.1
        assert max_speed == 1.0
        assert safe_angle == 0.2
        assert dangerous_angle == 0.8
        assert stop_on_shock == True

        mqtt_config.publish(topic='test/car/config/throttle/min', payload="0.4", qos=1) \
            .wait_for_publish()
        mqtt_config.publish(topic='test/car/config/throttle/max', payload="0.5", qos=1) \
            .wait_for_publish()
        mqtt_config.publish(topic='test/car/config/throttle/angle/safe', payload="0.3", qos=1) \
            .wait_for_publish()
        mqtt_config.publish(topic='test/car/config/throttle/angle/dangerous', payload="0.7", qos=1) \
            .wait_for_publish()
        mqtt_config.publish(topic='test/car/config/throttle/compute_from_steering', payload="true", qos=1) \
            .wait_for_publish()
        mqtt_config.publish(topic='test/car/config/throttle/stop_on_shock', payload="false", qos=1) \
            .wait_for_publish()

        wait_all_mqtt_messages_consumed(
            f'mqtt-subscription-{throttle_config_controller_mqtt._mqtt_client_id}'
            f'qos{throttle_config_controller_mqtt.qos}')

        use_steering, min_speed, max_speed, safe_angle, dangerous_angle, stop_on_shock = throttle_config_controller_mqtt.run()
        assert use_steering == True
        assert min_speed == 0.4
        assert max_speed == 0.5
        assert safe_angle == 0.3
        assert dangerous_angle == 0.7
        assert stop_on_shock == False
