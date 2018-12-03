from pathlib import Path
from time import sleep

import cv2
import pytest
from pytest import fixture

from donkeycar import Vehicle
from donkeycar.parts.camera import VideoCamera
from donkeycar.parts.img_process import ConvertToGrayPart, ThresholdPart, HistogramPart, BlurPart, CannyPart, \
    BoundingBoxPart
from donkeycar.parts.road import RoadConfigController, RoadPart, RoadDebugPart
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
    vehicle.register(BoundingBoxPart(input_img_key=ConvertToGrayPart.IMG_GRAY_RAW,
                                     output_img_key=ConvertToGrayPart.IMG_GRAY_RAW))
    vehicle.register(HistogramPart())
    vehicle.register(ThresholdPart())
    vehicle.register(BlurPart(input_key=ThresholdPart.IMG_THRESHOLD, output_key='img/blur'))
    vehicle.register(CannyPart(input_img_key='img/blur', output_img_key='img/canny'))
    # vehicle.register(HoughPart(input_img_key='img/canny', output_img_key='img/hough'))
    vehicle.register(RoadPart(road_config, input_img_type='img/canny'))
    vehicle.register(RoadDebugPart())

    # vehicle.register(BoundingBoxPart(input_img_key=IMG_GRAY_RAW, output_img_key=IMG_GRAY_RAW))
    # vehicle.register(HistogramPart())
    # vehicle.register(GraySelectorPart())
    # vehicle.register(ThresholdController(config=ThresholdConfigController(limit_min=180, limit_max=200,
    # threshold_default = 190,
    # threshold_delta = 10,
    # threshold_dynamic = False, mqtt_enable = False)))
    # vehicle.register(DilatePart())
    # vehicle.register(RoadPart(road_config))
    # vehicle.register(RoadDebugPart())
    return vehicle


@pytest.mark.skip
def test_renqder(vehicle: Vehicle) -> None:
    while True:
        vehicle.update_parts()
        gray = vehicle.mem.get(vehicle.parts[1]['outputs'])[0]
        gray2 = vehicle.mem.get(vehicle.parts[4]['outputs'])[0]
        blur = vehicle.mem.get(vehicle.parts[5]['outputs'])[0]
        canny = vehicle.mem.get(vehicle.parts[6]['outputs'])[0]
        # hough = vehicle.mem.get(vehicle.parts[6]['outputs'])[0]
        road = vehicle.mem.get(vehicle.parts[8]['outputs'])[0]
        # threshold = vehicle.mem.get(vehicle.parts[6]['outputs'])[0]
        # dilate = vehicle.mem.get(vehicle.parts[7]['outputs'])[0]
        cv2.imshow('gray', gray)
        cv2.imshow('gray2', gray2)
        cv2.imshow('blur', blur)
        cv2.imshow('canny', canny)
        # cv2.imshow('hough', hough)
        cv2.imshow('road', road)
        # cv2.imshow('threshold', threshold)
        # cv2.imshow('dilate', dilate)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if cv2.waitKey(1) & 0xFF == ord(' '):
            while True:
                if cv2.waitKey(1) & 0xFF == ord(' '):
                    break
        sleep(0.01)
