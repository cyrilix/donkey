import os
from pathlib import Path

import pytest

import donkeycar as dk
from donkeycar.parts.camera import ImageListCamera
from donkeycar.templates.fousduvolant_base import BaseVehicle
from donkeycar.tests.conftest import wait_port_open


class MalabilleCar(BaseVehicle):
    def _configure_car_hardware(self, cfg):
        pass

    def _configure_camera(self, cfg):
        base_path = str(os.path.join(Path(__file__).parent, '..', '..'))
        self.register(ImageListCamera(path_mask=base_path + '/test_parts/*.jpg'))

    def _configure_arduino(self, cfg):
        pass


@pytest.fixture(name='car')
def fixture_car(mqtt_address: (str, int)):
    wait_port_open(host=mqtt_address[0], port=mqtt_address[1])

    cfg = dk.load_config(config_path=str(os.path.join(Path(__file__).parent, '../templates/config_defaults.py')))
    car = MalabilleCar(cfg=cfg)
    yield car
    car.stop()


def test_run(car: MalabilleCar, caplog):
    car.start(rate_hz=20, max_loop_count=10)
    for record in caplog.records:
        assert record.levelname != 'CRITICAL'
        assert record.levelname != 'ERROR'
