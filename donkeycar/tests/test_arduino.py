import time
from queue import Queue
from threading import Thread
from typing import Iterator

import pytest

from donkeycar.parts.arduino import SerialPart, DRIVE_MODE_USER, DRIVE_MODE_PILOT


class TestSerialPart:

    @pytest.fixture(name='arduino')
    def serial_arduino(self) -> Queue:
        return Queue(maxsize=1)

    @pytest.fixture(name='serial_part')
    def serial_part_fixture(self, arduino: Queue, monkeypatch) -> Iterator[SerialPart]:
        monkeypatch.setattr('serial.Serial.open', lambda x: x)
        monkeypatch.setattr('serial.Serial.readline', lambda x: arduino.get(block=True))
        serial_part = SerialPart()

        thread = Thread(target=serial_part.update)
        thread.start()
        yield serial_part

        serial_part.shutdown()
        arduino.put(b'\n')
        thread.join(0.01)

    def test_read_serial(self, serial_part: SerialPart, arduino: Queue) -> None:
        steering, throttle, user_mode = serial_part.run_threaded()
        assert None is steering
        assert None is throttle
        assert 'user' == user_mode

        channel_1 = 678
        channel_2 = 910
        channel_3 = 1112
        channel_4 = 1678
        channel_5 = 1910
        channel_6 = 112
        arduino.put(f'12345,{channel_1},{channel_2},{channel_3},{channel_4},{channel_5},{channel_6},50\n'
                    .encode(encoding='utf-8'))

        time.sleep(0.01)
        steering, throttle, user_mode = serial_part.run_threaded()
        assert channel_1 == steering
        assert channel_2 == throttle
        assert 'user' == user_mode

    def test_read_invalid_line(self, serial_part: SerialPart, arduino: Queue) -> None:
        steering, throttle, user_mode = serial_part.run_threaded()
        assert None is steering
        assert None is throttle
        assert DRIVE_MODE_USER == user_mode

        arduino.put(b'\ninvalid\n')
        time.sleep(0.001)
        steering, throttle, user_mode = serial_part.run_threaded()
        assert None is steering
        assert None is throttle
        assert DRIVE_MODE_USER == user_mode

        channel_1 = 678
        channel_2 = 910
        channel_3 = 1112
        channel_4 = 1678
        channel_5 = 1910
        channel_6 = 112
        arduino.put(f'12345,{channel_1},{channel_2},{channel_3},{channel_4},{channel_5},{channel_6},50\n'
                    .encode(encoding='utf-8'))

        time.sleep(0.001)
        steering, throttle, user_mode = serial_part.run_threaded()
        assert channel_1 == steering
        assert channel_2 == throttle
        assert DRIVE_MODE_USER == user_mode

        arduino.put(b'\ninvalid\n')
        time.sleep(0.001)
        steering, throttle, user_mode = serial_part.run_threaded()
        assert channel_1 == steering
        assert channel_2 == throttle
        assert DRIVE_MODE_USER == user_mode

    def test_switch_user_mode(self, serial_part: SerialPart, arduino: Queue) -> None:
        _, _, user_mode = serial_part.run_threaded()
        assert DRIVE_MODE_USER == user_mode

        # Switch off
        channel_6 = 998
        arduino.put(f'12345,123,123,123,123,123,{channel_6},50\n'.encode(encoding='utf-8'))

        time.sleep(0.001)
        _, _, user_mode = serial_part.run_threaded()
        assert DRIVE_MODE_USER == user_mode

        # Switch on
        channel_6 = 1987
        arduino.put(f'12345,123,123,123,123,123,{channel_6},50\n'.encode(encoding='utf-8'))
        time.sleep(0.001)
        _, _, user_mode = serial_part.run_threaded()
        assert DRIVE_MODE_PILOT == user_mode

        channel_6 = 1850
        arduino.put(f'12345,123,123,123,123,123,{channel_6},50\n'.encode(encoding='utf-8'))
        time.sleep(0.001)
        _, _, user_mode = serial_part.run_threaded()
        assert DRIVE_MODE_PILOT == user_mode

        # Switch off
        channel_6 = 1003
        arduino.put(f'12345,123,123,123,123,123,{channel_6},50\n'.encode(encoding='utf-8'))
        time.sleep(0.001)
        _, _, user_mode = serial_part.run_threaded()
        assert DRIVE_MODE_USER == user_mode
