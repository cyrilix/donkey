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
        assert steering == 0.0
        assert throttle == 0.0
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
        assert steering != 0.0
        assert throttle != 0.0
        assert 'user' == user_mode

    def test_read_invalid_line(self, serial_part: SerialPart, arduino: Queue) -> None:
        steering, throttle, user_mode = serial_part.run_threaded()
        assert steering == 0.0
        assert throttle == 0.0
        assert DRIVE_MODE_USER == user_mode

        arduino.put(b'\ninvalid\n')
        time.sleep(0.001)
        steering, throttle, user_mode = serial_part.run_threaded()
        assert steering == 0.0
        assert throttle == 0.0
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
        assert steering != 0
        assert steering != 0
        assert DRIVE_MODE_USER == user_mode

        arduino.put(b'\ninvalid\n')
        time.sleep(0.001)
        new_steering, new_throttle, new_user_mode = serial_part.run_threaded()
        assert new_steering == steering
        assert new_throttle == throttle
        assert new_user_mode == user_mode

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

    def test_radio_angle(self, serial_part: SerialPart, arduino: Queue) -> None:
        angle, _, _ = serial_part.run_threaded()
        assert 0.0 == angle

        # Over Left
        channel_1 = 99
        arduino.put(f'123,{channel_1},123,123,123,123,123,50\n'.encode(encoding='utf-8'))

        time.sleep(0.001)
        angle, _, _ = serial_part.run_threaded()
        assert angle == -1.0

        # Left
        channel_1 = 998
        arduino.put(f'123,{channel_1},123,123,123,123,123,50\n'.encode(encoding='utf-8'))

        time.sleep(0.001)
        angle, _, _ = serial_part.run_threaded()
        assert -1.0 < angle < -0.9

        # Middle
        channel_1 = 1450
        arduino.put(f'123,{channel_1},123,123,123,123,123,50\n'.encode(encoding='utf-8'))

        time.sleep(0.001)
        angle, _, _ = serial_part.run_threaded()
        assert -0.1 < angle < 0.1

        # Right
        channel_1 = 1958
        arduino.put(f'123,{channel_1},123,123,123,123,123,50\n'.encode(encoding='utf-8'))

        time.sleep(0.001)
        angle, _, _ = serial_part.run_threaded()
        assert 1.0 > angle > 0.9

        # Over Right
        channel_1 = 2998
        arduino.put(f'123,{channel_1},123,123,123,123,123,50\n'.encode(encoding='utf-8'))

        time.sleep(0.001)
        angle, _, _ = serial_part.run_threaded()
        assert angle == 1.0

    def test_radio_throttle(self, serial_part: SerialPart, arduino: Queue) -> None:
        throttle, _, _ = serial_part.run_threaded()
        assert 0.0 == throttle

        # Over down
        channel_2 = 99
        arduino.put(f'123,123,{channel_2},123,123,123,123,50\n'.encode(encoding='utf-8'))

        time.sleep(0.001)
        throttle, _, _ = serial_part.run_threaded()
        assert throttle == -1.0

        # Left
        channel_2 = 998
        arduino.put(f'123,123,{channel_2},123,123,123,123,50\n'.encode(encoding='utf-8'))

        time.sleep(0.001)
        _, throttle, _ = serial_part.run_threaded()
        assert -1.0 < throttle < -0.9

        # Stop
        channel_2 = 1450
        arduino.put(f'123,123,{channel_2},123,123,123,123,50\n'.encode(encoding='utf-8'))

        time.sleep(0.001)
        _, throttle, _ = serial_part.run_threaded()
        assert -0.1 < throttle < 0.1

        # Up
        channel_2 = 1948
        arduino.put(f'123,123,{channel_2},123,123,123,123,50\n'.encode(encoding='utf-8'))

        time.sleep(0.001)
        _, throttle, _ = serial_part.run_threaded()
        assert 1.0 > throttle > 0.9

        # Over up
        channel_2 = 2998
        arduino.put(f'123,123,{channel_2},123,123,123,123,50\n'.encode(encoding='utf-8'))

        time.sleep(0.001)
        _, throttle, _ = serial_part.run_threaded()
        assert throttle == 1.0
