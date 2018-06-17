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

        steering_pwm = 678
        throttle_pwm = 910
        user_mode_pwm = 1112
        arduino.put(f'12345,{steering_pwm},{throttle_pwm},{user_mode_pwm},50\n'.encode(encoding='utf-8'))

        time.sleep(0.01)
        steering, throttle, user_mode = serial_part.run_threaded()
        assert steering_pwm == steering
        assert throttle_pwm == throttle
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

        steering_pwm = 678
        throttle_pwm = 910
        user_mode_pwm = 1112
        arduino.put(f'12345,{steering_pwm},{throttle_pwm},{user_mode_pwm},50\n'.encode(encoding='utf-8'))

        time.sleep(0.001)
        steering, throttle, user_mode = serial_part.run_threaded()
        assert steering_pwm == steering
        assert throttle_pwm == throttle
        assert DRIVE_MODE_USER == user_mode

        arduino.put(b'\ninvalid\n')
        time.sleep(0.001)
        steering, throttle, user_mode = serial_part.run_threaded()
        assert steering_pwm == steering
        assert throttle_pwm == throttle
        assert DRIVE_MODE_USER == user_mode

    def test_toggle_user_mode(self, serial_part: SerialPart, arduino: Queue) -> None:
        _, _, user_mode = serial_part.run_threaded()
        assert DRIVE_MODE_USER == user_mode

        # Button released
        user_mode_pwm = 1050
        arduino.put(f'12345,123,123,{user_mode_pwm},50\n'.encode(encoding='utf-8'))

        time.sleep(0.001)
        _, _, user_mode = serial_part.run_threaded()
        assert DRIVE_MODE_USER == user_mode

        # Push
        user_mode_pwm = 1600
        arduino.put(f'12345,123,123,{user_mode_pwm},50\n'.encode(encoding='utf-8'))
        time.sleep(0.001)
        _, _, user_mode = serial_part.run_threaded()
        assert DRIVE_MODE_PILOT == user_mode

        user_mode_pwm = 1800
        arduino.put(f'12345,123,123,{user_mode_pwm},50\n'.encode(encoding='utf-8'))
        time.sleep(0.001)
        _, _, user_mode = serial_part.run_threaded()
        assert DRIVE_MODE_PILOT == user_mode

        # Release
        user_mode_pwm = 1100
        arduino.put(f'12345,123,123,{user_mode_pwm},50\n'.encode(encoding='utf-8'))
        time.sleep(0.001)
        _, _, user_mode = serial_part.run_threaded()
        assert DRIVE_MODE_PILOT == user_mode

        # Push
        user_mode_pwm = 1600
        arduino.put(f'12345,123,123,{user_mode_pwm},50\n'.encode(encoding='utf-8'))
        time.sleep(0.001)
        _, _, user_mode = serial_part.run_threaded()
        assert DRIVE_MODE_USER == user_mode

        user_mode_pwm = 1800
        arduino.put(f'12345,123,123,{user_mode_pwm},50\n'.encode(encoding='utf-8'))
        time.sleep(0.001)
        _, _, user_mode = serial_part.run_threaded()
        assert DRIVE_MODE_USER == user_mode

        # Release
        user_mode_pwm = 1100
        arduino.put(f'12345,123,123,{user_mode_pwm},50\n'.encode(encoding='utf-8'))
        time.sleep(0.001)
        _, _, user_mode = serial_part.run_threaded()
        assert DRIVE_MODE_USER == user_mode
