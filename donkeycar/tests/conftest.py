import importlib as importlib
import logging
import os
from pathlib import Path
from time import sleep
from typing import Dict, List, Iterator

import cv2
import docker
import importlib as importlib
import pytest
import requests
from compose.cli.command import project_from_options
from compose.container import Container
from compose.project import Project
from compose.service import ImageType
from docker import DockerClient
from numpy import ndarray
from paho.mqtt import client as mqtt

DOCKER_COMPOSE_PROJECT = 'donkeycar'

DOCKER_SERVICES = ['mqtt']

logger = logging.getLogger(__name__)

learning = pytest.mark.skipif(importlib.util.find_spec('tensorflow') is None, reason="Learning lib not installed")


learning = pytest.mark.skipif(importlib.util.find_spec('tensorflow') is None, reason="Learning lib not installed")

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
            r = requests.get('http://{0}:{1}/api/queues/%2F/{2}?columns=messages'.format(host, port, queue),
                             auth=('guest', 'guest'))
            content = r.json()
            if 'messages' in content and content['messages'] == 0:
                return
        except:
            logger.debug('Wait port %s open', port)
        sleep(0.5)


@pytest.fixture(name='mqtt_config')
def fixture_mqtt_config(mqtt_address: (str, int)):
    mqtt_client = mqtt.Client(client_id="test-push-config", clean_session=False, userdata=None, protocol=mqtt.MQTTv311)
    mqtt_client.connect(host=mqtt_address[0], port=mqtt_address[1], keepalive=60)
    mqtt_client.loop_start()
    yield mqtt_client
    mqtt_client.loop_stop()
    mqtt_client.disconnect()


def _load_img(img):
    path = os.path.join(_base_path(), "donkeycar/tests/data/", img)
    return cv2.imread(path)


def _load_img_gray(img):
    return cv2.cvtColor(_load_img(img), cv2.COLOR_RGB2GRAY)


@pytest.fixture(scope='session')
def img_straight_line() -> ndarray:
    return _load_img("straight_line_1.jpg")


@pytest.fixture(scope='session')
def img_straight_line_gray() -> ndarray:
    return _load_img_gray("straight_line_1.jpg")


@pytest.fixture(scope='session')
def img_turn_right_gray() -> ndarray:
    return _load_img_gray("turn_right.jpg")


@pytest.fixture(scope='session')
def img_straight_line_binarized_150_200() -> ndarray:
    return _load_img_gray('straight_line_binarized_150_200.jpg')


@pytest.fixture(scope="session")
def docker_project() -> Project:
    """
    Builds the Docker project if necessary, once per session.

    Returns the project instance, which can be used to start and stop
    the Docker containers.
    """
    path = _base_path()
    docker_compose = Path(path)

    if docker_compose.is_dir():
        docker_compose /= "docker-compose.yml"

    if not docker_compose.is_file():
        raise ValueError(
            "Unable to find `{0}` for integration tests.".format(docker_compose),
        )

    project = project_from_options(
        project_dir=str(docker_compose.parent),
        options={"--file": [docker_compose.name],
                 '--project-name': DOCKER_COMPOSE_PROJECT}
    )
    project.build()

    return project


@pytest.fixture(scope='session')
def docker_containers(docker_project: Project) -> Iterator[Dict[str, Container]]:
    """
    Spins up a the containers for the Docker project and returns
    them.

    Note that this fixture's scope is a single test; the containers
    will be stopped after the test is finished.

    This is intentional; stopping the containers destroys local
    storage, so that the next test can start with fresh containers.
    """
    containers: List[Container] = docker_project.up(DOCKER_SERVICES)
    if not containers:
        raise ValueError("`docker-compose` didn't launch any containers!")
    containers_by_name = dict([(c.name, c) for c in containers])

    yield containers_by_name

    # Send container logs to stdout, so that they get included in
    # the test report.
    # https://docs.pytest.org/en/latest/capture.html
    for container in sorted(containers, key=lambda c: c.name):
        header = "Logs from {0}:".format(container.name)
        logger.info(header)
        logger.info("=" * len(header))
        logger.info(
            container.logs().decode("utf-8", errors="replace") or
            "(no logs)"
        )
        logger.info('')

    docker_project.down(ImageType.none, False)


def _base_path():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
    return path


@pytest.fixture(scope='session', name='docker_client')
def fixture_docker_client() -> Iterator[DockerClient]:
    client = docker.client.from_env()
    yield client
    client.close()


@pytest.fixture(scope='session')
def mqtt_address(docker_containers: Dict[str, Container], docker_client: DockerClient) -> (str, int):
    mqtt = docker_containers.get('{0}_mqtt_1'.format(DOCKER_COMPOSE_PROJECT))
    container = docker_client.containers.get(mqtt.id)

    # host = container.attrs['NetworkSettings']['Ports']['1883/tcp'][0]['HostPort']
    host = '127.0.0.1'
    port = int(container.attrs['NetworkSettings']['Ports']['1883/tcp'][0]['HostPort'])
    wait_port_open(host=host, port=port)

    return host, port
