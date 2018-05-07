from typing import List, Dict

import pytest

import donkeycar as dk
from donkeycar.parts.camera import ImageListCamera
from donkeycar.templates.fousduvolant_base import BaseVehicle
from donkeycar.tests.conftest import wait_port_open, NetworkInfo


class MalabilleCar(BaseVehicle):
    def _configure_car_hardware(self, cfg):
        pass

    def _configure_camera(self, cfg):
        camera = ImageListCamera(path_mask='test_parts/*.jpg')
        self.add(camera, outputs=['cam/image_array'], threaded=True)

    def _configure_arduino(self, cfg):
        pass


@pytest.fixture(name='car')
def fixture_car(docker_network_info: Dict[str, List[NetworkInfo]]):
    mqtt_service = docker_network_info["donkeycar_mqtt_1"][0]
    host = 'localhost'
    port = 1883
    wait_port_open(host=host, port=port)

    cfg = dk.load_config(config_path='donkeycar/templates/config_defaults.py')
    return MalabilleCar(cfg=cfg)


def test_run(car: MalabilleCar, caplog):
    car.start(rate_hz=20, max_loop_count=10)
    for record in caplog.records:
        assert record.levelname != 'CRITICAL'
        assert record.levelname != 'ERROR'
