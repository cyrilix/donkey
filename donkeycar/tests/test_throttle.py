from paho.mqtt.client import Client
from pytest import fixture

from donkeycar.parts.throttle import ThrottleControllerSteeringBased, ThrottleControllerFixedSpeed, \
    ThrottleConfigController
from donkeycar.tests.conftest import wait_all_mqtt_messages_consumed


@fixture(name='throttle_config_controller')
def fixture_throttle_config_controller() -> ThrottleConfigController:
    return ThrottleConfigController(mqtt_enable=False,
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


class TestThrottleControllerSteeringBased:
    @fixture(name='throttle_controller_angle')
    def fixture_throttle_controller_angle(self, throttle_config_controller):
        return ThrottleControllerSteeringBased(throttle_config_controller=throttle_config_controller)

    def test_throttle_with_min_angle(self, throttle_controller_angle: ThrottleControllerSteeringBased):
        assert throttle_controller_angle.run(angle=0.0) == 0.8
        assert throttle_controller_angle.run(angle=0.01) == 0.8
        assert throttle_controller_angle.run(angle=-0.01) == 0.8

    def test_throttle_with_max_angle(self, throttle_controller_angle: ThrottleControllerSteeringBased):
        assert throttle_controller_angle.run(angle=1.0) == 0.4
        assert throttle_controller_angle.run(angle=-1.0) == 0.4

    def test_throttle_with_intermediate_angle(self, throttle_controller_angle: ThrottleControllerSteeringBased):
        assert throttle_controller_angle.run(angle=0.5) == 0.6
        assert throttle_controller_angle.run(angle=-0.5) == 0.6
        assert throttle_controller_angle.run(angle=0.8) == 0.72
        assert throttle_controller_angle.run(angle=-0.8) == 0.72
        assert throttle_controller_angle.run(angle=0.2) == 0.48
        assert throttle_controller_angle.run(angle=-0.2) == 0.48


class TestThrottleConfigController:
    @fixture(name='throttle_config_controller_mqtt')
    def fixture_throttle_config_controller_mqtt(self, mqtt_address):
        throttle_config = ThrottleConfigController(min_speed=0.1, max_speed=1, safe_angle=0.2, dangerous_angle=0.8,
                                                   use_steering=False,
                                                   mqtt_enable=True,
                                                   mqtt_hostname=mqtt_address[0],
                                                   mqtt_port=mqtt_address[1],
                                                   mqtt_qos=1,
                                                   mqtt_client_id='donkey-config-throttle-',
                                                   mqtt_topic='test/car/config/throttle/#')

        wait_all_mqtt_messages_consumed(f'mqtt-subscription-{throttle_config._mqtt_client_id}'
                                        f'qos{throttle_config.qos}')
        yield throttle_config
        throttle_config.shutdown()

    def test_values(self):
        throttle_config_controller = ThrottleConfigController(mqtt_enable=False,
                                                              min_speed=0.1,
                                                              max_speed=1.0,
                                                              safe_angle=0.2,
                                                              dangerous_angle=0.8,
                                                              use_steering=False)
        use_steering, min_speed, max_speed, safe_angle, dangerous_angle = throttle_config_controller.run()
        assert use_steering == False
        assert min_speed == 0.1
        assert max_speed == 1.0
        assert safe_angle == 0.2
        assert dangerous_angle == 0.8

    def test_modify_config_with_mqtt(self, throttle_config_controller_mqtt: ThrottleConfigController,
                                     mqtt_config: Client):
        use_steering, min_speed, max_speed, safe_angle, dangerous_angle = throttle_config_controller_mqtt.run()
        assert use_steering == False
        assert min_speed == 0.1
        assert max_speed == 1.0
        assert safe_angle == 0.2
        assert dangerous_angle == 0.8

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

        wait_all_mqtt_messages_consumed(
            f'mqtt-subscription-{throttle_config_controller_mqtt._mqtt_client_id}'
            f'qos{throttle_config_controller_mqtt.qos}')

        use_steering, min_speed, max_speed, safe_angle, dangerous_angle = throttle_config_controller_mqtt.run()
        assert use_steering == True
        assert min_speed == 0.4
        assert max_speed == 0.5
        assert safe_angle == 0.3
        assert dangerous_angle == 0.7
