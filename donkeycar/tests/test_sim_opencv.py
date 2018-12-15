from pathlib import Path
from time import sleep

import cv2
import pytest
from pytest import fixture

from donkeycar import Vehicle
from donkeycar.parts.angle import RoadEllipseDebugPart, AngleRoadPart
from donkeycar.parts.camera import VideoCamera
from donkeycar.parts.img_process import ConvertToGrayPart, HistogramPart, GraySelectorPart, BlurPart, BoundingBoxPart, \
    DilatePart, ThresholdPart, CannyPart
from donkeycar.parts.road import RoadDebugPart, RoadPart, RoadConfigController, ComponentRoadPart
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
    vehicle.register(ComponentRoadPart())
    vehicle.register(AngleRoadPart())
    vehicle.register(RoadEllipseDebugPart())
    return vehicle


@pytest.mark.skip
def test_render(vehicle: Vehicle) -> None:
    while True:
        vehicle.update_parts()
        gray = vehicle.mem.get([ConvertToGrayPart.IMG_GRAY_RAW])[0]
        threshold = vehicle.mem.get([ThresholdPart.IMG_THRESHOLD])[0]
        canny = vehicle.mem.get([CannyPart.IMG_CANNY])[0]
        road = vehicle.mem.get([RoadDebugPart.IMG_ROAD])[0]
        road_ellipse = vehicle.mem.get([RoadEllipseDebugPart.IMG_ROAD_ELLIPSE])[0]
        cv2.imshow('gray', gray)
        cv2.imshow('threshold', threshold)
        cv2.imshow('canny', canny)
        cv2.imshow('road', road)
        if road_ellipse is not None:
            cv2.imshow('road_ellipse', road_ellipse)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if cv2.waitKey(1) & 0xFF == ord(' '):
            while True:
                if cv2.waitKey(1) & 0xFF == ord(' '):
                    break
        sleep(0.01)
