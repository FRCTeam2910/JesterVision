import io
import time
from base_camera import BaseCamera
import cv2 as cv
from queue import Queue
# import numpy
import cv2 as cv

class Camera(BaseCamera):
    frameQueue = None

    def __init__(self, frameQueue):
        Camera.frameQueue = frameQueue
        super(Camera, self).__init__()

    @staticmethod
    def frames():
        while True:
            yield Camera.frameQueue.get()