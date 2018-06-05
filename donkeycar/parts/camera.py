import glob
import logging
import os
import time
from typing import List

import cv2
import numpy as np
from PIL import Image

from donkeycar.parts.part import ThreadedPart

CAM_IMAGE = 'cam/image_array'

logger = logging.getLogger(__name__)


class BaseCamera:

    def run_threaded(self):
        return self.frame


class PiCamera(BaseCamera, ThreadedPart):

    def __init__(self, resolution=(120, 160), framerate=20, rotation=0):
        from picamera.array import PiRGBArray
        from picamera import PiCamera
        resolution = (resolution[1], resolution[0])
        # initialize the camera and stream
        self.camera = PiCamera()  # PiCamera gets resolution (height, width)
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.camera.rotation = rotation
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,
                                                     format="rgb", use_video_port=True)

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.on = True

        logger.info('PiCamera loaded.. .warming camera')
        time.sleep(2)

    def run(self):
        f = next(self.stream)
        frame = f.array
        self.rawCapture.truncate(0)
        return frame

    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.rawCapture.truncate(0)

            # if the thread indicator variable is set, stop the thread
            if not self.on:
                break

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        logger.info('stoping PiCamera')
        time.sleep(.5)
        self.stream.close()
        self.rawCapture.close()
        self.camera.close()

    def get_inputs_keys(self) -> List[str]:
        return []

    def get_outputs_keys(self) -> List[str]:
        return [CAM_IMAGE]


class WebcamCV(BaseCamera):
    """
    Use webcam with opencv
    """

    def __init__(self, resolution=(120, 160), framerate=20, index=0):
        super().__init__()
        import numpy as np
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.frame = np.zeros(resolution)

    def update(self):
        while True:
            # Capture frame-by-frame
            ret, self.frame = self.cap.read()

    def run_threaded(self):
        return self.frame

    def shutdown(self):
        # indicate that the thread should be stopped
        logger.info('stoping Webcam')
        self.cap.release()
        cv2.destroyAllWindows()
        time.sleep(.5)


class Webcam(BaseCamera):
    def __init__(self, resolution=(120, 160), framerate=20):
        import pygame.camera

        super().__init__()

        pygame.init()
        pygame.camera.init()
        l = pygame.camera.list_cameras()
        resolution = (resolution[1], resolution[0])
        self.cam = pygame.camera.Camera(l[0], resolution, "RGB")
        self.resolution = resolution
        self.cam.start()
        self.framerate = framerate

        # initialize variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.on = True

        logger.info('WebcamVideoStream loaded.. .warming camera')

        time.sleep(2)

    def update(self):
        from datetime import datetime
        import pygame.image
        while self.on:
            start = datetime.now()

            if self.cam.query_image():
                # snapshot = self.cam.get_image()
                # self.frame = list(pygame.image.tostring(snapshot, "RGB", False))
                snapshot = self.cam.get_image()
                snapshot1 = pygame.transform.scale(snapshot, self.resolution)
                self.frame = pygame.surfarray.pixels3d(
                    pygame.transform.rotate(pygame.transform.flip(snapshot1, True, False), 90))

            stop = datetime.now()
            s = 1 / self.framerate - (stop - start).total_seconds()
            if s > 0:
                time.sleep(s)

        self.cam.stop()

    def run_threaded(self):
        return self.frame

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        logger.info('stoping Webcam')
        time.sleep(.5)


class MockCamera(BaseCamera):
    '''
    Fake camera. Returns only a single static frame
    '''

    def __init__(self, resolution=(160, 120), image=None):
        if image is not None:
            self.frame = image
        else:
            self.frame = Image.new('RGB', resolution)

    def update(self):
        pass

    def shutdown(self):
        pass


class ImageListCamera(BaseCamera, ThreadedPart):
    """
    Use the images from a tub as a fake camera output
    """

    def __init__(self, path_mask='~/d2/data/**/*.jpg'):
        self.image_filenames = glob.glob(os.path.expanduser(path_mask), recursive=True)

        def get_image_index(fnm):
            sl = os.path.basename(fnm).split('_')
            return int(sl[0])

        '''
        I feel like sorting by modified time is almost always
        what you want. but if you tared and moved your data around,
        sometimes it doesn't preserve a nice modified time.
        so, sorting by image index works better, but only with one path.
        '''
        self.image_filenames.sort(key=get_image_index)
        # self.image_filenames.sort(key=os.path.getmtime)
        self.num_images = len(self.image_filenames)
        print(self.image_filenames[:10])
        self.i_frame = 0
        self.frame = None
        self.update()

    def update(self):
        pass

    def run_threaded(self):
        if self.num_images > 0:
            self.i_frame = (self.i_frame + 1) % self.num_images
            self.frame = np.array(Image.open(self.image_filenames[self.i_frame]))

        return self.frame

    def get_inputs_keys(self) -> List[str]:
        return []

    def get_outputs_keys(self) -> List[str]:
        return [CAM_IMAGE]
