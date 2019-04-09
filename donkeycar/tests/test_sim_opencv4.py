from pathlib import Path
from time import sleep

import cv2
from pytest import fixture, mark

from donkeycar import Vehicle
from donkeycar.parts.angle import RoadEllipseDebugPart, AngleRoadPart
from donkeycar.parts.camera import VideoCamera
from donkeycar.parts.img_process import ConvertToGrayPart, ThresholdPart, CannyPart
from donkeycar.parts.road import RoadDebugPart, RoadConfigController, ComponentRoadPart, ComponentRoadPart2
from donkeycar.parts.throttle import ThrottleDebugPart, ThrottleEllipsePart, ThrottleConfigController
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


@fixture(name='throttle_config')
def fixture_throttle_config() -> ThrottleConfigController:
    return ThrottleConfigController(mqtt_enable=False,
                                    min_speed=0.1,
                                    max_speed=1.0,
                                    safe_angle=0.1,
                                    dangerous_angle=0.8,
                                    use_steering=False)


@fixture(name='vehicle')
def fixture_vehicle(video_camera: VideoCamera, road_config: RoadConfigController,
                    throttle_config: ThrottleConfigController) -> Vehicle:
    vehicle = Vehicle()
    vehicle.register(video_camera)
    vehicle.register(ComponentRoadPart2())
    vehicle.register(AngleRoadPart())
    vehicle.register(ThrottleEllipsePart(throttle_config_controller=throttle_config))
    vehicle.register(RoadEllipseDebugPart())
    vehicle.register(ThrottleDebugPart(input_img_key=RoadEllipseDebugPart.IMG_ROAD_ELLIPSE))
    return vehicle


#@mark.skip
def test_render(vehicle: Vehicle) -> None:
    while True:
        vehicle.update_parts()
        gray = vehicle.mem.get([ConvertToGrayPart.IMG_GRAY_RAW])[0]
        threshold = vehicle.mem.get([ThresholdPart.IMG_THRESHOLD])[0]
        road = vehicle.mem.get([RoadDebugPart.IMG_ROAD])[0]
        road_ellipse = vehicle.mem.get([RoadEllipseDebugPart.IMG_ROAD_ELLIPSE])[0]
        cv2.imshow('gray', gray)
        cv2.imshow('threshold', threshold)
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
