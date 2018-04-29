import logging
from time import sleep
from typing import Dict, List

import pytest
import requests
from paho.mqtt import client as mqtt
from pytest_docker_compose import NetworkInfo

logger = logging.getLogger(__name__)

pytest_plugins = ["docker_compose"]


def wait_port_open(host: str, port: int):
    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while True:
        result = sock.connect_ex((host, port))
        if result == 0:
            return
        sleep(0.5)


def wait_all_mqtt_messages_consumed(queue, host='localhost', port=15672):
    while True:
        try:
            r = requests.get(f'http://{host}:{port}/api/queues/%2F/{queue}?columns=messages',
                             auth=('guest', 'guest'))
            content = r.json()
            if 'messages' in content and content['messages'] == 0:
                return
        except:
            logger.debug(f'Wait port {port} open')
        sleep(0.5)


@pytest.fixture(name='mqtt_config')
def fixture_mqtt_config(docker_network_info: Dict[str, List[NetworkInfo]]):
    wait_port_open('localhost', 1883)
    mqtt_client = mqtt.Client(client_id="test-push-config", clean_session=False, userdata=None, protocol=mqtt.MQTTv311)
    mqtt_client.connect(host='localhost', port=1883, keepalive=60)
    mqtt_client.loop_start()
    yield mqtt_client
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
