import json
import logging
from time import sleep
from typing import Iterator
from unittest.mock import Mock

import pytest
from paho.mqtt.client import MQTTMessage, Client
from paho.mqtt.subscribe import simple

from donkeycar.parts.arduino import DRIVE_MODE_USER
from donkeycar.parts.mqtt import on_drive_message, MqttDrive, MqttMetricsPublisher, MultiProcessingMetringPublisher
from donkeycar.tests.conftest import wait_all_mqtt_messages_consumed
from donkeycar.vehicle import MetricsPublisher

logger = logging.getLogger(__name__)


class TestOnDriveMessage:
    def test_on_drive_message_user(self, monkeypatch):
        msg = MQTTMessage()
        msg._topic = 'config/drive/mode'.encode('utf-8')
        msg.payload = "user".encode('utf-8')

        request = Mock()
        monkeypatch.setattr("requests.post", request)

        mqtt_drive = Mock(user_mode='local')
        on_drive_message(client=Mock(), msg=msg, userdata=mqtt_drive)

        request.assert_called_once_with(json='{"angle": 0, "throttle": 0, "drive_mode": "user"}',
                                        url='http://127.0.0.1:8887/drive')
        assert mqtt_drive.user_mode == 'user'

    def test_on_drive_message_local(self, monkeypatch):
        msg = MQTTMessage()
        msg._topic = 'config/drive/mode'.encode('utf-8')
        msg.payload = 'local'.encode('utf-8')

        request = Mock()
        monkeypatch.setattr("requests.post", request)

        mqtt_drive = Mock(user_mode='user')
        on_drive_message(client=Mock(), msg=msg, userdata=mqtt_drive)

        request.assert_called_once_with(json='{"angle": 0, "throttle": 0, "drive_mode": "local"}',
                                        url='http://127.0.0.1:8887/drive')
        assert mqtt_drive.user_mode == 'local'


class TestDriveConfigController:
    @pytest.fixture(name='mqtt_drive')
    def fixture_mqtt_drive(self, mqtt_address: (str, int)) -> MqttDrive:
        mqtt_drive = MqttDrive(mqtt_enable=True,
                               mqtt_hostname=mqtt_address[0],
                               mqtt_port=mqtt_address[1],
                               mqtt_qos=1,
                               mqtt_client_id='donkey-config-drive-',
                               mqtt_topic='test/car/config/drive/#')

        wait_all_mqtt_messages_consumed(f'mqtt-subscription-{mqtt_drive._mqtt_client_id}'
                                        f'qos{mqtt_drive.qos}')
        yield mqtt_drive
        mqtt_drive.shutdown()

    def test_values(self, mqtt_drive: MqttDrive, mqtt_config: Client):
        user_mode = mqtt_drive.run()
        assert user_mode == 'user'

        mqtt_config.publish(topic='test/car/config/drive/mode', payload="local", qos=1) \
            .wait_for_publish()

        wait_all_mqtt_messages_consumed(
            f'mqtt-subscription-{mqtt_drive._mqtt_client_id}'
            f'qos{mqtt_drive.qos}')

        user_mode = mqtt_drive.run()
        assert user_mode == 'local'


class TestMultiProcessing:

    @pytest.fixture(name='metrics')
    def fixture_metrics_multiprocessing(self, mqtt_address: (str, int)) -> Iterator[MqttMetricsPublisher]:
        mqtt_publisher = MultiProcessingMetringPublisher(client_id='test_multiprocessing',
                                                         mqtt_address=mqtt_address,
                                                         topic='test/multiprocessing',
                                                         qos=1)
        sleep(5)
        yield mqtt_publisher
        mqtt_publisher.shutdown()

    @pytest.mark.skip(reason='Test slow and possible deadlock')
    def test_mqtt_metrics(self, mqtt_address: (str, int), metrics: MetricsPublisher):
        metrics.publish({'key': 'value', 'user/mode': DRIVE_MODE_USER})
        message: MQTTMessage = simple(hostname=mqtt_address[0], port=mqtt_address[1], topics='#', msg_count=1)
        logger.info("Messages: %s", message.payload)
        payload = json.loads(message.payload)

        assert 'value' == payload['payload']['key']
        assert 'user' == payload['payload']['user/mode']
