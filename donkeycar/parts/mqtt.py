import base64
import json
import logging
import time
from abc import abstractmethod
from datetime import datetime
from multiprocessing import Process, Queue
from typing import Dict, Any, Callable, List
from typing import Tuple

import numpy
import requests
from paho.mqtt import client as mqtt
from paho.mqtt.client import MQTTMessage, Client

from donkeycar import utils
from donkeycar.parts.part import Part
from donkeycar.vehicle import MetricsPublisher

USER_MODE = 'user/mode'

logger = logging.getLogger(__name__)


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, numpy.ndarray):
            return obj.tolist()
        if isinstance(obj, numpy.number):
            return obj.item()
        return json.JSONEncoder.default(self, obj)


class MultiProcessingMetringPublisher(MetricsPublisher):

    def __init__(self, topic: str = 'car', client_id: str = 'test', mqtt_address: Tuple[str, int] = ('localhost', 1883),
                 mqtt_user: str = 'guest', mqtt_password: str = 'guest', qos=0) -> None:
        self.queue = Queue()
        self._process = Process(target=run_process,
                                args=(self.queue, topic, client_id, mqtt_address, mqtt_user, mqtt_password, qos))
        self._process.start()

    def publish(self, values: Dict[str, Any]):
        logger.debug("Put message %s", values)
        self.queue.put(values, block=True)

    def shutdown(self):
        self.queue.close()
        self._process.terminate()


def run_process(queue: Queue, topic: str, client_id: str, mqtt_address: Tuple[str, int], mqtt_user: str,
                mqtt_password: str, qos: int):
    try:
        publisher = MqttMetricsPublisher(topic=topic, client_id=client_id, hostname=mqtt_address[0],
                                         port=mqtt_address[1],
                                         username=mqtt_user, password=mqtt_password,
                                         qos=qos)
        while True:
            logger.debug('Wait message')
            metrics = queue.get(block=True)
            logger.debug("Receive msg %s", metrics)

            publisher.publish(metrics)
    except:
        logger.exception("Unexpected error")


class MqttMetricsPublisher(MetricsPublisher):

    def __init__(self, topic='car', hostname='localhost', port=1883,
                 client_id="parts_publish", username=None, password=None, qos=1, publish_all_events=True):
        self._qos = qos
        self._previous_mode = None
        self.record_time = 0
        self.start_time = time.time()
        self._reset_tub_name()

        self._mqtt_client = mqtt.Client(client_id=client_id + "-", clean_session=False, userdata=None,
                                        protocol=mqtt.MQTTv311)
        if username:
            self._mqtt_client.username_pw_set(username=username, password=password)
        self._mqtt_client.connect(hostname, port, 60)
        self._mqtt_client.loop_start()
        self._topic = topic
        self._publish_all_events = publish_all_events

    def run(self, *args):
        """
        API function needed to use as a Donkey part.

        Accepts values, pairs them with their inputs keys and send them to mqtt broker.
        """
        record = dict(zip(self._inputs, args))
        self.publish(record)

    def publish(self, record):
        self.record_time = int(time.time() - self.start_time)
        self._send_record(record)

    def _send_record(self, data):
        json_data = {}
        self._current_idx += 1

        if not self._publish_all_events and USER_MODE not in data:
            logger.warning("'%s' part not defined", USER_MODE)
            return

        user_mode = data[USER_MODE]
        if user_mode != self._previous_mode:
            self._reset_tub_name()
            self._previous_mode = user_mode

        if not self._publish_all_events and "user" == user_mode:
            # Don't send event when autonomous drive not active
            return

        for key, val in data.items():

            if isinstance(val, tuple):
                json_data[key] = list(val)
            elif isinstance(val, bytes):
                name = self.make_file_name(key, ext='.jpg')
                message = self._build_image_message(image_name=name, img_content=val, part=key)
                json_data[key] = name
                self._publish(message, topic=self._topic + "/image/" + key)
            elif isinstance(val, numpy.ndarray):
                img_content = utils.arr_to_binary(val)
                name = self.make_file_name(key, ext='.jpg')
                message = self._build_image_message(image_name=name, img_content=img_content, part=key)
                json_data[key] = name
                self._publish(message, topic=self._topic + "/image/" + key)
            elif not val or isinstance(val, (str, float, int, bool, list)):
                json_data[key] = val

            else:
                logger.warning('Tub does not know what to do with this for key %s: %s', key, val)
                return
        msg = {
            'content_type': 'application/json',
            'application_headers': {
                'name': self._get_json_record_name(),
                'tub_name': self._tub_name,
                'index': self._current_idx
            },
            'payload': json_data
        }
        self._publish(message=msg, topic=self._topic + '/parts')

    def _build_image_message(self, image_name, part, img_content: bytes) -> Dict[str, Any]:
        return {'payload': base64.standard_b64encode(img_content).decode('utf-8'),
                'content_type': 'image/jpeg',
                'application_headers': {
                    'name': image_name,
                    'part': part,
                    'tub_name': self._tub_name,
                    'index': self._current_idx
                }
                }

    def _get_json_record_name(self):
        return 'record_{0}.json'.format(self._current_idx)

    def make_file_name(self, key, ext='.png'):
        name = '_'.join([key, str(self._current_idx), ext])
        name = name.replace('/', '-')
        return name

    def _publish(self, message, topic):
        self._mqtt_client.publish(payload=json.dumps(message, cls=NumpyEncoder), topic=topic,
                                  qos=self._qos)

    def shutdown(self):
        if self._mqtt_client:
            self._mqtt_client.loop_stop()
            self._mqtt_client.disconnect()

    def _reset_tub_name(self):
        self._tub_name = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._current_idx = 0


class MqttController(Part):
    def __init__(self, mqtt_client_id: str, mqtt_enable: bool, mqtt_hostname: str, mqtt_password: str, mqtt_port: int,
                 mqtt_qos: int, mqtt_topic: str, mqtt_username: str,
                 on_message: Callable[[Client, Any, MQTTMessage], None]):
        self._mqtt_client_id = mqtt_client_id
        if mqtt_enable:
            logger.info("Init mqtt connection to %s topic", mqtt_topic)
            self.topic = mqtt_topic
            self.qos = mqtt_qos
            self._mqtt_client = mqtt.Client(client_id=mqtt_client_id, clean_session=False, userdata=self,
                                            protocol=mqtt.MQTTv311)

            if mqtt_username:
                self._mqtt_client.username_pw_set(username=mqtt_username, password=mqtt_password)
            self._mqtt_client.user_data_set(userdata=self)
            self._mqtt_client.on_connect = _on_connect
            self._mqtt_client.on_message = on_message
            self._mqtt_client.connect(mqtt_hostname, mqtt_port, 60)
            self._mqtt_client.loop_start()

    def shutdown(self):
        if self._mqtt_client:
            self._mqtt_client.loop_stop()
            self._mqtt_client.disconnect()

    @abstractmethod
    def get_inputs_keys(self) -> List[str]:
        pass

    @abstractmethod
    def get_outputs_keys(self) -> List[str]:
        pass


def _on_connect(client: Client, userdata: MqttController, flags, rc: int):
    logger.info("Connected with result code %s", rc)
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(userdata.topic, userdata.qos)
    logger.info("Subscribe to %s topic", userdata.topic)


class MqttDrive(MqttController):

    def __init__(self, mqtt_enable: bool = True, mqtt_topic: str = 'config/drive/#',
                 mqtt_hostname: str = 'localhost', mqtt_port: int = 1883, mqtt_client_id: str = "donkey-config-mqtt-",
                 mqtt_username: str = None, mqtt_password: str = None, mqtt_qos: int = 0):
        super().__init__(mqtt_client_id, mqtt_enable, mqtt_hostname, mqtt_password, mqtt_port, mqtt_qos, mqtt_topic,
                         mqtt_username, on_message=on_drive_message)
        self.user_mode = "user"

    def run(self) -> str:
        """
        :return: parts
        * user/mode
        """
        return self.user_mode

    def get_inputs_keys(self) -> List[str]:
        return []

    def get_outputs_keys(self) -> List[str]:
        return [USER_MODE]


def on_drive_message(client: Client, userdata: MqttDrive, msg: MQTTMessage):
    logger.info('new message: %s', msg.topic)
    if msg.topic.endswith("drive/mode"):
        new_value = msg.payload.decode('utf-8')
        logger.info("Update drive mode from %s to %s", userdata.user_mode, new_value)
        userdata.user_mode = new_value
        requests.post(url='http://127.0.0.1:8887/drive',
                      json={'angle': 0, 'throttle': 0, 'drive_mode': new_value})
    else:
        logger.warning("Unexpected msg for topic %s", msg.topic)
