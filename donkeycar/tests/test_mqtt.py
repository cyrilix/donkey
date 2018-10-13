import json
import logging
from time import sleep
from typing import Iterator

import pytest
from paho.mqtt.client import MQTTMessage
from paho.mqtt.subscribe import simple

from donkeycar.parts.arduino import DRIVE_MODE_USER
from donkeycar.parts.mqtt import MqttMetricsPublisher, MultiProcessingMetringPublisher
from donkeycar.vehicle import MetricsPublisher

logger = logging.getLogger(__name__)


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
