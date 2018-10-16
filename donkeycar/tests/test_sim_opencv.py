from pathlib import Path
from time import sleep

import cv2
from pytest import fixture

from donkeycar import Vehicle
from donkeycar.parts.camera import VideoCamera
from donkeycar.parts.img_process import ConvertToGrayPart, HistogramPart, GraySelectorPart, BlurPart, IMG_GRAY_RAW, \
    BoundingBoxPart, DilatePart
from donkeycar.parts.road import RoadDebugPart, RoadPart, RoadConfigController
from donkeycar.parts.threshold import ThresholdController, ThresholdConfigController
from donkeycar.tests.conftest import _base_path


@fixture(name='video')
def fixture_video() -> Path:
    return _base_path().joinpath("donkeycar/tests/data", 'video2.mp4')


@fixture(name='video_camera')
def fixture_video_camera(video: Path) -> VideoCamera:
    vc = VideoCamera(video)
    yield vc
    vc.shutdown()


@fixture(name='road_config')
def fixture_road_config() -> RoadConfigController:
    return RoadConfigController(enable=True,
                                canny_threshold1=180,
                                canny_threshold2=200,
                                kernel_size=4,
                                mqtt_enable=False)


@fixture(name='vehicle')
def fixture_vehicle(video_camera: VideoCamera, road_config: RoadConfigController) -> Vehicle:
    vehicle = Vehicle()
    vehicle.register(video_camera)
    vehicle.register(ConvertToGrayPart())
    vehicle.register(BlurPart(input_key=IMG_GRAY_RAW, output_key=IMG_GRAY_RAW))
    vehicle.register(BoundingBoxPart(input_img_key=IMG_GRAY_RAW, output_img_key=IMG_GRAY_RAW))
    vehicle.register(HistogramPart())
    vehicle.register(GraySelectorPart())
    vehicle.register(ThresholdController(config=ThresholdConfigController(limit_min=180, limit_max=200,
                                                                          threshold_default=190,
                                                                          threshold_delta=10,
                                                                          threshold_dynamic=False, mqtt_enable=False)))
    vehicle.register(DilatePart())
    vehicle.register(RoadPart(road_config))
    vehicle.register(RoadDebugPart())
    return vehicle


def test_render(vehicle: Vehicle) -> None:
    while True:
        vehicle.update_parts()
        gray = vehicle.mem.get(vehicle.parts[5]['outputs'])[0]
        threshold = vehicle.mem.get(vehicle.parts[6]['outputs'])[0]
        dilate = vehicle.mem.get(vehicle.parts[7]['outputs'])[0]
        road = vehicle.mem.get(vehicle.parts[9]['outputs'])[0]
        cv2.imshow('gray', gray)
        cv2.imshow('threshold', threshold)
        cv2.imshow('dilate', dilate)
        cv2.imshow('road', road)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if cv2.waitKey(1) & 0xFF == ord(' '):
            while True:
                if cv2.waitKey(1) & 0xFF == ord(' '):
                    break
        sleep(0.01)
