# -*- coding: utf-8 -*-

import pytest
from donkeycar.parts.keras import KerasPilot, KerasCategorical, KerasIMU
from donkeycar.parts.keras import default_categorical, default_imu, default_n_linear
from donkeycar.tests.conftest import learning


# content of ./test_smtpsimple.py


@pytest.fixture
@learning
def pilot():
    kp = KerasPilot()
    return kp


@learning
def test_pilot_types(pilot):
    assert 1 == 1


@learning
def test_categorical():
    kc = KerasCategorical()
    assert kc.model is not None


@learning
def test_categorical_with_model():
    kc = KerasCategorical(default_categorical())
    assert kc.model is not None


@learning
def test_def_n_linear_model():
    model = default_n_linear(10)
    assert model is not None


@learning
def test_imu_model():
    model = default_imu(2, 7)
    assert model is not None


@learning
def test_KerasIMU():
    k = KerasIMU(num_outputs=2, num_imu_inputs=7)
    assert k.model is not None
