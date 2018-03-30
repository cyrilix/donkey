import json
import logging
from datetime import datetime

import numpy
import time
from amqpy import Connection, Message

from donkeycar import utils

logger = logging.getLogger(__name__)


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, numpy.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


class TubAmqpWriter:
    def __init__(self, inputs, input_types, queue_name, exchange='car', hostname='localhost'):
        self._previous_mode = None
        self.record_time = 0
        self._input_types = input_types
        self.start_time = time.time()
        self._inputs = inputs
        self._exchange = exchange
        self._queue_name = queue_name
        self._init_channel(hostname)
        self._reset_tub_name()

    def _init_channel(self, hostname):
        self._connection = Connection(host=hostname)
        self._channel = self._connection.channel()
        self._channel.exchange_declare(exchange=self._exchange, exch_type='direct')
        self._channel.queue_declare(queue=self._queue_name, auto_delete=False)
        self._channel.queue_bind(self._queue_name, exchange=self._exchange, routing_key=self._queue_name)

    def run(self, *args):
        """
        API function needed to use as a Donkey part.

        Accepts values, pairs them with their inputs keys and send them to amqp broker.
        """
        assert len(self._input_types) == len(args)

        self.record_time = int(time.time() - self.start_time)
        record = dict(zip(self._inputs, args))
        logger.info(args)
        logger.info(record)
        self._send_record(record)

    def _send_record(self, data):
        json_data = {}
        self._current_idx += 1

        if "user/mode" not in data:
            logger.warning("'user/mode' part not defined")
            return

        user_mode = data["user/mode"]
        if user_mode != self._previous_mode:
            self._reset_tub_name()
            self._previous_mode = user_mode

        if "user" == user_mode:
            # Don't send event when autonomous drive not active
            return

        for key, val in data.items():
            typ = self._input_types.get(key)

            if typ in ['str', 'float', 'int', 'boolean', 'list']:
                json_data[key] = val
            elif typ is 'tuple':
                json_data[key] = list(val)
            elif typ is 'image':
                name = self.make_file_name(key, ext='.jpg')
                message = Message(val,
                                  content_type='image/jpeg',
                                  application_headers={'name': name,
                                                       'tub_name': self._tub_name,
                                                       'index': self._current_idx})
                json_data[key] = name
                self._publish(message)

            elif typ == 'image_array':
                name = self.make_file_name(key, ext='.jpg')
                img_content = utils.arr_to_binary(val)
                message = Message(img_content,
                                  content_type='image/jpeg',
                                  application_headers={'name': name,
                                                       'tub_name': self._tub_name,
                                                       'index': self._current_idx})
                json_data[key] = name
                self._publish(message)

            else:
                logger.warning('Tub does not know what to do with this type %s for key %s', typ, key)
                return
        self._publish(Message(json.dumps(json_data, cls=NumpyEncoder),
                              content_type='application/json',
                              application_headers={'name': self._get_json_record_name(),
                                                   'tub_name': self._tub_name,
                                                   'index': self._current_idx}))

    def _get_json_record_name(self):
        return 'record_{0}.json'.format(self._current_idx)

    def make_file_name(self, key, ext='.png'):
        name = '_'.join([key, str(self._current_idx), ext])
        name = name.replace('/', '-')
        return name

    def _publish(self, message):
        self._channel.basic_publish(msg=message, exchange=self._exchange, routing_key=self._queue_name)

    def shutdown(self):
        if self._connection:
            self._connection.close()

    def _reset_tub_name(self):
        self._tub_name = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._current_idx = 0
