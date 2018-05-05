from typing import Dict, List

import pytest
from paho.mqtt.client import Client

from donkeycar.parts.throttle import ThrottleControllerSteeringBased, ThrottleControllerFixedSpeed, \
    ThrottleConfigController
from donkeycar.tests.conftest import wait_port_open, wait_all_mqtt_messages_consumed, NetworkInfo


@pytest.fixture(name='throttle_config_controller')
def fixture_throttle_config_controller() -> ThrottleConfigController:
    return ThrottleConfigController(mqtt_enable=False,
                                    stop_on_shock=False,
                                    min_speed=0.4,
                                    max_speed=0.8,
                                    safe_angle=0.1,
                                    dangerous_angle=0.8,
                                    use_steering=False)


class TestThrottleControllerFixedSpeed:

    def test_run(self, throttle_config_controller):
        throttle_controller = ThrottleControllerFixedSpeed(throttle_config_controller=throttle_config_controller)
        assert throttle_controller.run() == 0.4
        assert throttle_controller.run() == throttle_config_controller.min_speed

    def test_throttle_with_shock(self, throttle_config_controller):
        controller_fixed_speed = ThrottleControllerFixedSpeed(throttle_config_controller=throttle_config_controller)
        assert controller_fixed_speed.run(shock=False) == 0.4
        throttle_config_controller.stop_on_shock = True
        assert controller_fixed_speed.run(shock=True) == 0.0
        assert controller_fixed_speed.run(shock=False) == 0.0


@pytest.fixture(name='throttle_controller_angle')
def fixture_throttle_controller_angle(throttle_config_controller):
    return ThrottleControllerSteeringBased(throttle_config_controller=throttle_config_controller)


class TestThrottleControllerSteeringBased:

    def test_throttle_with_min_angle(self, throttle_controller_angle: ThrottleControllerSteeringBased):
        assert throttle_controller_angle.run(angle=0.0, shock=False) == 0.8
        assert throttle_controller_angle.run(angle=0.01, shock=False) == 0.8
        assert throttle_controller_angle.run(angle=-0.01, shock=False) == 0.8

    def test_throttle_with_max_angle(self, throttle_controller_angle: ThrottleControllerSteeringBased):
        assert throttle_controller_angle.run(angle=1.0, shock=False) == 0.4
        assert throttle_controller_angle.run(angle=-1.0, shock=False) == 0.4

    def test_throttle_with_intermediate_angle(self, throttle_controller_angle: ThrottleControllerSteeringBased):
        assert throttle_controller_angle.run(angle=0.5, shock=False) == 0.6
        assert throttle_controller_angle.run(angle=-0.5, shock=False) == 0.6
        assert throttle_controller_angle.run(angle=0.8, shock=False) == 0.72
        assert throttle_controller_angle.run(angle=-0.8, shock=False) == 0.72
        assert throttle_controller_angle.run(angle=0.2, shock=False) == 0.48
        assert throttle_controller_angle.run(angle=-0.2, shock=False) == 0.48

    def test_throttle_with_shock(self, throttle_controller_angle: ThrottleControllerSteeringBased):
        throttle_controller_angle._throttle_config_controller.stop_on_shock = True
        assert throttle_controller_angle.run(angle=1.0, shock=False) > 0.1
        assert throttle_controller_angle.run(angle=1.0, shock=True) == 0.0
        assert throttle_controller_angle.run(angle=1.0, shock=False) == 0.


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
                                               mqtt_client_id='donkey-config-throttle-',
                                               mqtt_topic='test/car/config/throttle/#')

    wait_all_mqtt_messages_consumed(f'mqtt-subscription-{throttle_config._mqtt_client_id}'
                                    f'qos{throttle_config.qos}')
    return throttle_config


class TestThrottleConfigController:

    def test_values(self):
        throttle_config_controller = ThrottleConfigController(mqtt_enable=False,
                                                              stop_on_shock=True,
                                                              min_speed=0.1,
                                                              max_speed=1.0,
                                                              safe_angle=0.2,
                                                              dangerous_angle=0.8,
                                                              use_steering=False)
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
