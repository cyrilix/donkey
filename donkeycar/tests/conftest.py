import logging
import os
import typing
from pathlib import Path
from typing import Dict, List

import cv2
import pytest
import requests
from compose.cli.command import project_from_options
from compose.container import Container
from compose.project import Project
from compose.service import ImageType
from numpy import ndarray
from paho.mqtt import client as mqtt
from time import sleep

logger = logging.getLogger(__name__)


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
            r = requests.get(f'http://{host}:{port}/api/queues/%2F/{queue}?columns=messages',
                             auth=('guest', 'guest'))
            content = r.json()
            if 'messages' in content and content['messages'] == 0:
                return
        except:
            logger.debug(f'Wait port {port} open')
        sleep(0.5)


class NetworkInfo:
    """
    Container for info about how to connect to a service exposed by a
    Docker container.
    """
    container_port: typing.Text
    """
    Port (and usually also protocol name) exposed internally on the
    container.
    """

    hostname: typing.Text
    """
    Hostname to use when accessing this service.
    """

    host_port: int
    """
    Port number to use when accessing this service.
    """

    def __init__(
            self,
            container_port: typing.Text,
            hostname: typing.Text,
            host_port: int,
    ):
        super().__init__()

        self.container_port = container_port
        self.hostname = hostname
        self.host_port = host_port


@pytest.fixture(name='mqtt_config')
def fixture_mqtt_config(mqtt_service: NetworkInfo):
    wait_port_open('localhost', 1883)
    mqtt_client = mqtt.Client(client_id="test-push-config", clean_session=False, userdata=None, protocol=mqtt.MQTTv311)
    mqtt_client.connect(host='localhost', port=1883, keepalive=60)
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
def img_straight_line_gray() -> ndarray:
    return _load_img_gray("straight_line_1.jpg")


@pytest.fixture(scope='session')
def img_turn_right_gray() -> ndarray:
    return _load_img_gray("turn_right.jpg")


@pytest.fixture(scope='session')
def img_straight_line_binarized_150_200() -> ndarray:
    return _load_img_gray('straight_line_binarized_150_200.jpg')


@pytest.fixture(scope='session')
def docker_containers(docker_project: Project):
    """
    Spins up a the containers for the Docker project and returns
    them.

    Note that this fixture's scope is a single test; the containers
    will be stopped after the test is finished.

    This is intentional; stopping the containers destroys local
    storage, so that the next test can start with fresh containers.
    """
    containers: typing.List[Container] = docker_project.up()

    if not containers:
        raise ValueError("`docker-compose` didn't launch any containers!")

    yield containers

    # Send container logs to stdout, so that they get included in
    # the test report.
    # https://docs.pytest.org/en/latest/capture.html
    for container in sorted(containers, key=lambda c: c.name):
        header = f"Logs from {container.name}:"
        print(header)
        print("=" * len(header))
        print(
            container.logs().decode("utf-8", errors="replace") or
            "(no logs)"
        )
        print()

    docker_project.down(ImageType.none, False)


@pytest.fixture(scope='session')
def docker_network_info(docker_containers: typing.List[Container]):
    """
    Returns hostnames and exposed port numbers for each container,
    so that tests can interact with them.
    """
    return {
        container.name: [
            NetworkInfo(
                container_port=container_port,
                hostname=port_config["HostIp"] or "localhost",
                host_port=port_config["HostPort"],
            )

            # Example::
            #
            #   {'8181/tcp': [{'HostIp': '', 'HostPort': '8182'}]}
            for container_port, port_configs
            in container.get("HostConfig.PortBindings").items()

            for port_config in port_configs
        ]

        for container in docker_containers
    }


@pytest.fixture(scope="session")
def docker_project():
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
            f"Unable to find `{docker_compose}` for integration tests.",
        )

    project = project_from_options(
        project_dir=str(docker_compose.parent),
        options={"--file": [docker_compose.name]},
    )
    project.build()

    return project


def _base_path():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
    return path


@pytest.fixture(scope='session')
def mqtt_service(docker_network_info: Dict[str, List[NetworkInfo]]) -> NetworkInfo:
    return docker_network_info["donkeycar_mqtt_1"][0]
