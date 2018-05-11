import logging

import pytest

import donkeycar as dk
from donkeycar.parts.transform import Lambda


@pytest.fixture()
def vehicle():
    v = dk.Vehicle()

    def f(): return 1

    l = Lambda(f, outputs=['test_out'])
    v.register(l)
    return v


def test_create_vehicle():
    v = dk.Vehicle()
    assert v.parts == []


def test_add_part():
    v = dk.Vehicle()

    def f():
        return 1

    l = Lambda(f)
    v.add(l, outputs=['test_out'])
    assert len(v.parts) == 1


def test_vehicle_run(vehicle):
    vehicle.start(rate_hz=20, max_loop_count=2)
    assert vehicle is not None


def test_vehicle_register_part(vehicle):
    def f_mock(input):
        pass

    vehicle.register(Lambda(f_mock, inputs=["input1"], outputs=["output1"]))
    logging.info(str(vehicle.parts))
    assert len(vehicle.parts) == 2
    assert vehicle.parts[1]['inputs'] == ['input1']
    assert vehicle.parts[1]['outputs'] == ['output1']
