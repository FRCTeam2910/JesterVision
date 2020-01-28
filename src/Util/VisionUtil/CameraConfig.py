import cv2 as cv
import numpy as np
import json

class CameraConfig:
    def __init__(self, cameraConfigFilePath):
        cameraConfigFile = open(cameraConfigFilePath, 'r')
        cameraConfig = json.load(cameraConfigFile)
        self.resolution = tuple(cameraConfig['resolution'])
        self.framerate = cameraConfig['fps']
        self.distortionCoefficients = np.array(cameraConfig['distortionCoefficients'])
        self.cameraMatrix = np.array(cameraConfig['cameraMatrix'])
        # [w, h]
        self.sensorSize = np.array(cameraConfig['sensorSize'])
        _, _, self.focalLengthMM, _, _ = cv.calibrationMatrixValues(self.cameraMatrix, self.resolution, self.sensorSize[0], self.sensorSize[1])
        self.focalLengthPX = (self.focalLengthMM / self.sensorSize[0]) * self.resolution[0]
        self.frameMidpoint = np.array([self.resolution[0] / 2, self.resolution[1] / 2])
        self.frameSize = self.resolution[0] * self.resolution[1]

    def getResolution(self):
        return self.resolution

    def getFramerate(self):
        return self.framerate

    def getCameraMatrix(self):
        return self.cameraMatrix

    def getDistortionCoefficients(self):
        return self.distortionCoefficients

    def getFrameMidpoint(self):
        return self.frameMidpoint

    def getFrameSize(self):
        return self.frameSize