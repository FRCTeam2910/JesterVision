import os
import cv2 as cv
import numpy as np
import math
from queue import Queue
import threading
from camera_memory import Camera
from picamera import PiCamera
import time
import random
from struct import pack, unpack
import json
import Util.VisionUtil.CameraConfig as CameraConfig
import Util.VisionUtil.VisionUtil as VisionUtil
import Util.VisionUtil.Contour as Contour
import Util.VisionUtil.ContourGroup as ContourGroup
import Util.MathUtil.Vector3 as Vector3
import Util.MathUtil.Rotation3 as Rotation3
import Util.MathUtil.RigidTransform3 as RigidTransform3
import io
import socket
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)

HOST = '127.0.0.1'
PORT = 5801
STREAMING_PORT = 5802

CameraConfig = CameraConfig.CameraConfig
VisionUtil = VisionUtil.VisionUtil
Contour = Contour.Contour
ContourGroup = ContourGroup.ContourGroup
Vector3 = Vector3.Vector3
Rotation3 = Rotation3.Rotation3
RigidTransform3 = RigidTransform3.RigidTransform3

def bytesToPipeline(pipelineBytes):
    pipeline = json.loads(pipelineBytes.decode('utf-8'))
    if pipeline['vals']['objectPoints'] is not None:
        pipeline['vals']['objectPoints'] = np.array(pipeline['vals']['objectPoints'], dtype=np.float32)
    for val in pipeline['vals']:
        if (type(pipeline['vals'][val]) == list):
            pipeline['vals'][val] = tuple(pipeline['vals'][val])
    activePipeline = pipeline['activePipeline']
    pipeline = pipeline['vals']
    return activePipeline, pipeline

def bytesToPipelines(pipelinesBytes):
    data = json.loads(pipelinesBytes.decode('utf-8'))
    activePipeline = data['activePipeline']
    pipelines = data['pipelines']
    for pipeline in pipelines:
        if pipeline['objectPoints'] is not None:
            pipeline['objectPoints'] = np.array(pipeline['objectPoints'], dtype=np.float32)
    for pipeline in pipelines:
        for val in pipeline:
            if (type(pipeline[val]) == list):
                pipeline[val] = tuple(pipeline[val])
    return activePipeline, pipelines

# Some threading stuff
frameQueue = Queue(1)
loggingQueue = Queue(1)

pipelineLock = threading.Lock()
resultsLock = threading.Lock()

resultsReady = threading.Event()
restartPipeline = threading.Event()

# Some stuff for the pipeline
kernelShapes = {
    'rectangle': cv.MORPH_RECT,
    'cross': cv.MORPH_CROSS,
    'ellipse': cv.MORPH_ELLIPSE
}

pipelines = None
activePipeline = None

results = {
    'tv': 0,
    'tx': None,
    'ty': None,
    'ta': None,
    'ts': None,
    'tl': None,
    'tshort': None,
    'tlong': None,
    'thor': None,
    'tvert': None,
    'translation': None,
    'rotation': None
}

# Drawing functions

'''

    Default drawing colors (and there BGR values):
    Contours - pink (203, 192, 255)
    Contour Verticies - red (0, 0, 255)
    Contour Midpoint - (0, 100, 0)
    Reference Vector - green (0, 255, 0)
    Contour Label - blue (255, 0, 0)
    Rotated Bounding Box - cyan (255, 191, 0)
    Straight Bounding Box - yellow (0, 255, 255)

'''

font = cv.FONT_HERSHEY_SIMPLEX

def drawContours(img, contours):
    contours_temp = []
    for contour in contours:
        if (isinstance(contour, ContourGroup)):
            if contour.useConvexHull:
                contours_temp.append(contour.convexHull)
            else:
                group_contours = contour.contours
                for contour in group_contours:
                    contours_temp.append(contour.points)
        else:
            if contour.useConvexHull:
                contours_temp.append(contour.convexHull)
            else:
                contours_temp.append(contour.points)

    img_temp = np.array(img, copy=True)
    dst = cv.drawContours(img_temp, contours_temp, -1, (203, 192, 255), 2)
    return dst

def drawBoundingBoxes(img, contours):
    for contour in contours:
        cv.drawContours(img, contour.rotatedRect, 0, (255, 191, 0), 2)
        cv.rectangle(img, contour.boundingBoxUpperLeftPoint, contour.boundingBoxLowerRightPoint, (0, 255, 255), 2)

def labelContours(img, contours):
    for i in range(len(contours)):
        anchor = contours[i].boundingBoxUpperLeftPoint
        anchor = (anchor[0] - 20, anchor[1] - 5)
        cv.putText(img, str(i + 1), anchor, cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3, cv.LINE_AA)

def drawReferenceVector(img, contours):
    for contour in contours:
        point = contour.referenceVector.getPoint(18, True)
        cv.circle(img, contour.midpoint, 3, (0, 100, 0), 2, cv.LINE_AA)
        cv.line(img, contour.midpoint, point, (0, 255, 0), 2, cv.LINE_AA)

def labelVerticies(img, contours):
    pts = None
    for contour in contours:
        if contour.useConvexHull:
            pts = contour.convexHull
        else:
            pts = contour.vertices
        for x in range(len(pts)):
            cv.putText(img, str(x + 1), (int(pts[x][0][0]), int(pts[x][0][1])), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv.LINE_AA)

def drawPnPAxes(img, imgpts):
    origin = tuple(imgpts[0].ravel())
    img = cv.line(img, origin, tuple(imgpts[1].ravel()), (0,0,255), 3)
    img = cv.line(img, origin, tuple(imgpts[2].ravel()), (0,255,0), 3)
    img = cv.line(img, origin, tuple(imgpts[3].ravel()), (0,255,255), 3)
    return img

# Methods used by the pipeline
def findContours(img):
    contours, hierarchy = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    return contours

def processContours(contours, frameCenter, useConvexHull, numOfCorners):
    processed_contours = []
    for contour in contours:
        if (cv.contourArea(contour) > 20):
            cnt = Contour(contour, frameCenter, useConvexHull, numOfCorners)
            processed_contours.append(cnt)
    idec = []
    for contour in processed_contours:
        if (contour.rotatedRectArea > 20):
            idec.append(contour)
    return idec

def filterContours(contours, targetAreaRange, targetFullnessRange, aspectRatioRange, frameSize):
    filteredContours = []
    for contour in contours:
        cntTargetArea = contour.area / frameSize
        cntTargetFullness = contour.area / contour.rotatedRectArea
        cntAspectRatio = contour.boundingBoxAspectRatio

        withinTargetAreaRange = withinRange(cntTargetArea, targetAreaRange)
        withinTargetFullnessRange = withinRange(cntTargetFullness, targetFullnessRange)
        withinTargetAspectRatioRange = withinRange(cntAspectRatio, aspectRatioRange)

        if (withinTargetAreaRange and withinTargetFullnessRange and withinTargetAspectRatioRange):
            filteredContours.append(contour)

    if (len(filteredContours) == 0):
        return None
    
    return filteredContours

def sortContours(filteredContours, sortingMode):
    # left to right
    if (sortingMode == 'left'):
        return sorted(filteredContours, key=lambda cnt: cnt.midpoint[0])
    # right to left
    elif (sortingMode == 'right'):
        return sorted(filteredContours, key=lambda cnt: cnt.midpoint[0], reverse=True)
    # top to bottom
    elif (sortingMode == 'top'):
        return sorted(filteredContours, key=lambda cnt: cnt.midpoint[1])
    # bottom to top
    elif (sortingMode == 'bottom'):
        return sorted(filteredContours, key=lambda cnt: cnt.midpoint[1], reverse=True)
    # center outwards
    elif (sortingMode == 'center'):
        return sorted(filteredContours, key=lambda cnt: cnt.distanceToCenter)

def pairContours(sortedContours, intersectionLocation, targetAreaRange, targetFullnessRange, aspectRatioRange, sortingMode, frameCenter, frameSize, useConvexHull, numOfPairCorners):
    pairs = []
    if (intersectionLocation == 'neither'):
        for i in range(len(sortedContours)):
            refContour = sortedContours[i]
            j = i + 1
            while (j < len(sortedContours)):
                contour = sortedContours[j]
                _contours = [refContour, contour]
                pair = ContourGroup(_contours, frameCenter, useConvexHull, numOfPairCorners)
                pairs.append(pair)
                j = j + 1
    else:
        for i in range(len(sortedContours)):
            refContour = sortedContours[i]
            j = i + 1
            while (j < len(sortedContours)):
                contour = sortedContours[j]
                refContourRefVector = refContour.contourLine
                contourRefVector = contour.contourLine
                intersectionPoint = refContourRefVector.intersects(contourRefVector)
                if (intersectionPoint is not None):
                    intersectionPoint[1] = frameCenter[1] * 2 - intersectionPoint[1]
                    if (intersectionLocation == 'above' and intersectionPoint[1] < refContour.midpoint[1] and intersectionPoint[1] < contour.midpoint[1]):
                        _contours = [refContour, contour]
                        pair = ContourGroup(_contours, frameCenter, useConvexHull, numOfPairCorners)
                        pairs.append(pair)
                    elif (intersectionLocation == 'below' and intersectionPoint[1] > refContour.midpoint[1] and intersectionPoint[1] > contour.midpoint[1]):
                        _contours = [refContour, contour]
                        pair = ContourGroup(_contours, frameCenter, useConvexHull, numOfPairCorners)
                        pairs.append(pair)
                    elif (intersectionLocation == 'right' and intersectionPoint[0] > refContour.midpoint[0] and intersectionPoint[0] > contour.midpoint[0]):
                        _contours = [refContour, contour]
                        pair = ContourGroup(_contours, frameCenter, useConvexHull, numOfPairCorners)
                        pairs.append(pair)
                    elif (intersectionLocation == 'left' and intersectionPoint[0] < refContour.midpoint[0] and intersectionPoint[0] < contour.midpoint[0]):
                        _contours = [refContour, contour]
                        pair = ContourGroup(_contours, frameCenter, useConvexHull, numOfPairCorners)
                        pairs.append(pair)
                j = j + 1
    # Now filter the pairs
    filteredPairs = filterContours(pairs, targetAreaRange, targetFullnessRange, aspectRatioRange, frameSize)

    if filteredPairs is None:
        return None

    # Now sort the pairs
    sortedPairs = sortContours(filteredPairs, sortingMode)

    # Return the first pair in the list, which theoretically is the closest thing to what we want
    return [sortedPairs[0]]

# Other misc. helper methods
def withinRange(val, range):
    if (val > range[0] and val < range[1]):
        return True
    else:
        return False

# Pipeline thread
class Pipeline(threading.Thread):
    def __init__(self, owner, frameQueue, pipelineLock, resultsLock, cameraConfig, resultsReady, loggingQueue):
        super(Pipeline, self).__init__()
        self.owner = owner
        self.terminated = False
        self.frameQueue = frameQueue
        self.pipelineLock = pipelineLock
        self.resultsLock = resultsLock
        self.cameraConfig = cameraConfig
        self.resolution = cameraConfig.resolution
        self.resultsReady = resultsReady
        self.loggingQueue = loggingQueue
        self.recLightOn = True
        self.recLightTime = time.time()
        self.start()

    def run(self):
        while (self.owner.B.getbuffer().nbytes is 0):
            time.sleep(0.001)
        print('pipeline started!')
        while not self.terminated:
            # Mark the time at the beginning of the pipeline
            pipelineStart = round(time.time() * 1000, 2)

            # Grab the frame
            startGrabFrame = time.time()
            with self.owner.bufferLock:
                self.owner.B.seek(0, 0)
                frame = np.frombuffer(self.owner.B.read(self.resolution[0] * self.resolution[1] * 3), dtype=np.uint8)
                frame = frame.reshape((self.resolution[1], self.resolution[0], 3))
            endGrabFrame = time.time()

            # Grab the active pipeline
            startGrabPipeline = time.time()
            pipeline = {}
            with self.pipelineLock:
                global pipelines
                global activePipeline
                pipeline = pipelines[activePipeline]
                # pipeline = self.pipelines[self.activePipeline]
            endGrabPipeline = time.time()

            # Convert to HSV
            startToHSV = time.time()
            frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            endToHSV = time.time()

            # Threshold
            startThresh = time.time()
            frame_threshold = cv.inRange(frame_hsv, pipeline['hsvLow'], pipeline['hsvHigh'])
            endThresh = time.time()

            # Check if we should perform a erosion
            startMorph = time.time()
            if (pipeline['performErosion']):
                erosionKernel = cv.getStructuringElement(kernelShapes[pipeline['shapeOfErosionKernel']], pipeline['sizeOfErosionKernel'], anchor=pipeline['erosionKernelAnchor'])
                frame_threshold = cv.erode(frame_threshold, erosionKernel, anchor=pipeline['erosionAnchor'], iterations=pipeline['erosionIterations'])

            # Check if we should perform a dilation
            if (pipeline['performDilation']):
                dilationKernel = cv.getStructuringElement(kernelShapes[pipeline['shapeOfDilationKernel']], pipeline['sizeOfDilationKernel'], anchor=pipeline['dilationKernelAnchor'])
                frame_threshold = cv.dilate(frame_threshold, dilationKernel, anchor=pipeline['dilationAnchor'], iterations=pipeline['dilationIterations'])
            endMorph = time.time()

            # Find and process the contours within the image
            startFindContours = time.time()
            contours = findContours(frame_threshold)
            endFindContours = time.time()
            
            startProcessContours = time.time()
            processedContours = processContours(contours, self.cameraConfig.frameMidpoint, pipeline['contourConvexHull'], pipeline['numOfContourCorners'])
            endProcessContours = time.time()
            
            startFilterContours = time.time()
            filteredContours = filterContours(processedContours, pipeline['contourAreaRange'], pipeline['contourFullnessRange'], pipeline['contourAspectRatioRange'], self.cameraConfig.frameSize)
            endFilterContours = time.time()
            
            target = filteredContours
            
            startSortContours = endSortContours = startPairContours = endPairContours = None
            if filteredContours is not None:
                # Sort the contours
                startSortContours = time.time()
                sortedContours = sortContours(filteredContours, pipeline['contourSortingMode'])
                endSortContours = time.time()

                target = [sortedContours[0]]

                # Check if we should pair the contours
                if (pipeline['pairContours'] and len(sortedContours) >= 2):
                    startPairContours = time.time()
                    target = pairContours(sortedContours, pipeline['contourIntersectionLocation'], pipeline['targetAreaRange'], pipeline['targetFullnessRange'], pipeline['targetAspectRatioRange'], pipeline['targetSortingMode'], self.cameraConfig.frameMidpoint, self.cameraConfig.frameSize,  pipeline['targetConvexHull'], pipeline['numOfPairCorners'])
                    endPairContours = time.time()

            tx = ty = None
            if target is not None:
                tx = math.degrees(math.atan((target[0].midpoint[0] - (self.cameraConfig.frameMidpoint[0] - 0.5)) / self.cameraConfig.focalLengthPX))
                ty = -1.0 * math.degrees(math.atan((target[0].midpoint[1] - (self.cameraConfig.frameMidpoint[1] - 0.5)) / self.cameraConfig.focalLengthPX))

            startPnP = endPnP = None
            rigidTransform = None
            imgpts = None
            angleToTarget = None
            performPoseEstimation = pipeline['performPoseEstimation']
            if (performPoseEstimation and (target is not None) and (len(pipeline['objectPoints']) == len(target[0].vertices))):
                # print(target[0].vertices.dtype)
                startPnP = time.time()
                rigidTransform, imgpts = VisionUtil.getTranslation(self.cameraConfig.cameraMatrix, self.cameraConfig.distortionCoefficients, pipeline['objectPoints'], target[0].vertices, pipeline['poseEstimationRange'])
                angleToTarget = VisionUtil.getAngleToTarget(rigidTransform.rotation.rotationMatrix)
                endPnP = time.time()

            startDraw = time.time()
            cv.putText(frame, str(self.resolution[0]) + 'x' + str(self.resolution[1]) + ' @' + str(self.cameraConfig.framerate) + ' fps', (3, 11), font, 0.35, (255, 255, 255), 1, cv.LINE_AA)
            cv.putText(frame, 'Contours found: ' + str(len(contours)), (3, 23), font, 0.35, (255, 255, 255), 1, cv.LINE_AA)
            
            if (time.time() - self.recLightTime >= 1):
                self.recLightOn = not self.recLightOn
                self.recLightTime = time.time()

            if target is not None:
                # Draw and label the remaining bits
                frame = drawContours(frame, target)
                labelVerticies(frame, target)
                drawBoundingBoxes(frame, target)
                if (rigidTransform is not None):
                    cv.putText(frame, str(rigidTransform.translation), (3, 35), font, 0.35, (255, 255, 255), 1, cv.LINE_AA)
                    cv.putText(frame, str(rigidTransform.rotation), (3, 47), font, 0.35, (255, 255, 255), 1, cv.LINE_AA)
                    cv.putText(frame, str(np.degrees(angleToTarget)), (3, 59), font, 0.35, (255, 255, 255), 1, cv.LINE_AA)
                    frame = drawPnPAxes(frame, imgpts.clip(0, 7680))
                else:
                    drawReferenceVector(frame, target)

            if self.recLightOn:
                cv.circle(frame, (self.resolution[0] - 20, 20), 5, (0, 0, 255), thickness=10)
            endDraw = time.time()

            pipelineEnd = round(time.time() * 1000, 2)
            pipelineLatency = pipelineEnd - pipelineStart

            startPostResults = time.time()
            with self.resultsLock:
                global results
                if (target is not None):
                    results['tv'] = 1
                    results['tx'] = tx
                    results['ty'] = ty
                    results['ta'] = target[0].area
                    results['ts'] = target[0].rotation
                    results['tl'] = pipelineLatency
                    results['tshort'] = target[0].tshort
                    results['tlong'] = target[0].tlong
                    results['thor'] = target[0].boundingBoxWidth
                    results['tvert'] = target[0].boundingBoxHeight
                    if (rigidTransform is not None):
                        translation = rigidTransform.translation
                        rotation = rigidTransform.rotation
                        results['translation'] = (translation.x[0], translation.y[0], translation.z[0])
                        results['rotation'] = rotation.eulerAngles.tolist()
                    else:
                        results['translation'] = None
                        results['rotation'] = None
                else:
                    results['tv'] = 0
            endPostResults = time.time()

            startPostToStream = time.time()
            if not self.frameQueue.full():
                if (pipeline['streamType'] == 'results'):
                    self.frameQueue.put(frame)
                else:
                    self.frameQueue.put(frame_threshold)
                # print('pushed frame!')
            endPostToStream = time.time()

            self.resultsReady.set()

            # Calculate the time for each task, send it to the logger
            grabFrameTime = (endGrabFrame - startGrabFrame) * 1000
            grabPipelineTime = (endGrabPipeline - startGrabPipeline) * 1000
            toHSVTime = (endToHSV - startToHSV) * 1000
            threshTime = (endThresh - startThresh) * 1000
            morphTime = (endMorph - startMorph) * 1000
            findContoursTime = (endFindContours - startFindContours) * 1000
            processContoursTime = (endProcessContours - startProcessContours) * 1000
            filterContoursTime = (endFilterContours - startFilterContours) * 1000
            sortContoursTime = pairContoursTime = PnPTime = None
            if startSortContours is not None:
                sortContoursTime = (endSortContours - startSortContours) * 1000
            if startPairContours is not None:
                pairContoursTime = (endPairContours - startPairContours) * 1000
            if startPnP is not None:
                PnPTime = (endPnP - startPnP) * 1000
            drawTime = (endDraw - startDraw) * 1000
            postResultsTime = (endPostResults - startPostResults) * 1000
            postToStreamTime = (endPostToStream - startPostToStream) * 1000
            
            loggingData = {
                'id': 1,
                'grabFrameTime': grabFrameTime,
                'grabPipelineTime': grabPipelineTime,
                'toHSVTime': toHSVTime,
                'threshTime': threshTime,
                'morphTime': morphTime,
                'findContoursTime': findContoursTime,
                'processContoursTime': processContoursTime,
                'filterContoursTime': filterContoursTime,
                'sortContoursTime': sortContoursTime,
                'pairContoursTime': pairContoursTime,
                'PnPTime': PnPTime,
                'drawTime': drawTime,
                'postResultsTime': postResultsTime,
                'postToStreamTime': postToStreamTime,
                'pipelineLatency': pipelineLatency
            }
            self.loggingQueue.put(loggingData)
            time.sleep(0.001)

# This is our custom output object for the pipeline
class ProcessOutput(object):
    def __init__(self, frameQueue, pipelineLock, resultsLock, cameraConfig, resultsReady, loggingQueue):
        # Create an instance of the pipeline
        self.bufferLock = threading.Lock()
        self.A = io.BytesIO()
        self.B = io.BytesIO()
        self.pipeline = Pipeline(self, frameQueue, pipelineLock, resultsLock, cameraConfig, resultsReady, loggingQueue)
        self.loggingQueue = loggingQueue
        self.startGetData = time.time()

    def write(self, data):
        # Write the data
        endGetData = time.time()
        startWriteData = time.time()
        self.A.seek(0, 0)
        self.A.truncate(0)
        self.A.write(data)
        endWriteData = time.time()

        # Swap the identifiers
        startIdentifierSwap = time.time()
        with self.bufferLock:
             self.A, self.B = self.B, self.A
        endIdentifierSwap = time.time()

        # Calculate logging times and push to logger
        getDataTime = (endGetData - self.startGetData) * 1000
        writeDataTime = (endWriteData - startWriteData) * 1000
        identifierSwapTime = (endIdentifierSwap - startIdentifierSwap) * 1000

        loggingData = {
            'id': 2,
            'getDataTime': getDataTime,
            'writeDataTime': writeDataTime,
            'identifierSwapTime': identifierSwapTime
        }
        self.loggingQueue.put(loggingData)

        self.startGetData = time.time()

    def flush(self):
        self.pipeline.terminated = True
        self.pipeline.join()

class PipelineRunner(threading.Thread):
    def __init__(self, frameQueue, pipelineLock, resultsLock, restartPipeline, resultsReady, loggingQueue):
        super(PipelineRunner, self).__init__()
        self.frameQueue = frameQueue
        self.cameraConfig = None
        self.output = None
        self.cam = PiCamera()
        self.pipelineLock = pipelineLock
        self.resultsLock = resultsLock
        self.restartPipeline = restartPipeline
        self.resultsReady = resultsReady
        self.loggingQueue = loggingQueue
        self.start()

    def run(self):
        # Initially we use this event to wait on starting the PipelineRunner
        self.restartPipeline.wait()
        self.restartPipeline.clear()

        while True:
            # Start the camera
            self.cameraConfig = CameraConfig('/media/loggingDrive/cameraConfig.json')
            resolution = self.cameraConfig.getResolution()
            framerate = self.cameraConfig.getFramerate()
            cameraMatrix = self.cameraConfig.getCameraMatrix()
            distortionCoefficients = self.cameraConfig.getDistortionCoefficients()
            frameMidpoint = self.cameraConfig.getFrameMidpoint()
            frameSize = self.cameraConfig.getFrameSize()
            
            self.cam.resolution = resolution
            self.cam.framerate = framerate
            self.cam.iso = 400
            
            # Allow the gains to settle
            time.sleep(2)
            self.cam.shutter_speed = 2000
            self.cam.exposure_mode = 'off'
            self.cam.awb_mode = 'off'
            g = self.cam.awb_gains
            self.cam.awb_gains = g
            print('started camera')

            self.output = ProcessOutput(self.frameQueue, self.pipelineLock, self.resultsLock, self.cameraConfig, self.resultsReady, self.loggingQueue)
            self.cam.start_recording(self.output, format='bgr')
            while True:
                # Grab the current pipline
                startGrabPipeline = time.time()
                pipeline = {}
                with self.pipelineLock:
                    global pipelines
                    global activePipeline
                    pipeline = pipelines[activePipeline]
                endGrabPipeline = time.time()
                
                # Set sensor settings
                startSetSensor = time.time()
                self.cam.shutter_speed = pipeline['shutterSpeed'] * 1000
                self.cam.iso = pipeline['iso']
                self.cam.awb_gains = pipeline['awbGains']
                if pipeline['flipImg']:
                    self.cam.rotation = 180
                else:
                    self.cam.rotation = 0
                endSetSensor = time.time()

                # Set the LED ring
                startSetLEDs = time.time()
                if (pipeline['ledMode'] == 0):
                    # Turn the LED's off
                    GPIO.output(17, GPIO.LOW)
                    GPIO.output(18, GPIO.LOW)
                else:
                    # Turn the LED's on
                    GPIO.output(17, GPIO.HIGH)
                    GPIO.output(18, GPIO.HIGH)
                endSetLEDs = time.time()

                # Calculate times for tasks, push it to the logger
                grabPipelineTime = (endGrabPipeline - startGrabPipeline) * 1000
                setSensorTime = (endSetSensor - startSetSensor) * 1000
                setLEDsTime = (endSetLEDs - startSetLEDs) * 1000

                loggingData = {
                    'id': 0,
                    'grabPipelineTime': grabPipelineTime,
                    'setSensorTime': setSensorTime,
                    'setLEDsTime': setLEDsTime
                }

                self.loggingQueue.put(loggingData)

                # Chill out a bit
                time.sleep(0.01)

                if self.restartPipeline.is_set():
                    self.restartPipeline.clear()
                    break
            
            # Stop recording. This will call flush() on ProcessOutput, shutting down the pipeline properply. Also, stop_recording() will block until the pipeline has shutdown completely
            self.cam.stop_recording()
            print('shut down pipeline succfully!')

class IPCHandler(threading.Thread):
    def __init__(self, host, port, pipelineLock, resultsLock, resultsReady, restartPipeline, loggingQueue):
        super(IPCHandler, self).__init__()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.socket.bind((host, port))
        self.pipelineLock = pipelineLock
        
        self.resultsLock = resultsLock

        self.resultsReady = resultsReady
        self.restartPipeline = restartPipeline
        self.loggingQueue = loggingQueue
        self.start()

    def run(self):
        self.socket.listen()
        conn, addr = self.socket.accept()
        print('IPC Handler Connected!')
        with conn:
            # Get the pipeline for the first time
            lengthInBytes = conn.recv(8)
            (length,) = unpack('>Q', lengthInBytes)
            pipelinesBytes = b''
            while len(pipelinesBytes) < length:
                bytesLeftToRead = length - len(pipelinesBytes)
                pipelinesBytes+=conn.recv(1024 if bytesLeftToRead > 1024 else bytesLeftToRead)
            
            conn.sendall(b'\00')

            with self.pipelineLock:
                global pipelines
                global activePipeline
                activePipeline, pipelines = bytesToPipelines(pipelinesBytes)

            # Notify the pipeline the data is ready
            self.restartPipeline.set()

            while True:
                # Wait on the pipline to finish processing a frame
                self.resultsReady.wait()

                # Send the results of the pipeline over the socket
                startSendResults = time.time()
                with self.resultsLock:
                    global results
                    conn.sendall(json.dumps(results).encode('utf-8'))
                endSendResults = time.time()

                # Clear the event for next time, wait on the ack before proceeding
                startResultsAck = time.time()
                self.resultsReady.clear()
                ack = conn.recv(1)
                endResultsAck = time.time()

                # Recieve the pipeline and cameraConfig
                startRecvPipeline = time.time()
                pipelinesBytes = conn.recv(2048)
                conn.sendall(b'\00')
                endRecvPipeline = time.time()

                startRecvRestartPipeline = time.time()
                m_restartPipeline = conn.recv(1)
                endRecvRestartPipeline = time.time()

                startDecode = time.time()
                m_activePipeline, m_pipeline = bytesToPipeline(pipelinesBytes)
                with self.pipelineLock:
                    activePipeline = m_activePipeline
                    pipelines[activePipeline] = m_pipeline
                if (m_restartPipeline == b'\01'):
                    self.restartPipeline.set()
                endDecode = time.time()

                # Calculate the times for each task, push it to the logging queue
                sendResultsTime = (endSendResults - startSendResults) * 1000
                resultsAckTime = (endResultsAck - startResultsAck) * 1000
                recvPipelineTime = (endRecvPipeline - startRecvPipeline) * 1000
                recvRestartPipelineTime = (endRecvRestartPipeline - startRecvRestartPipeline) * 1000
                decodeTime = (endDecode - startDecode) * 1000
                
                loggingData = {
                    'id': 3, 
                    'sendResultsTime': sendResultsTime, 
                    'resultsAckTime': resultsAckTime, 
                    'recvPipelineTime': recvPipelineTime, 
                    'recvRestartPipelineTime': recvRestartPipelineTime,
                    'decodeTime': decodeTime
                }

                self.loggingQueue.put(loggingData)

                # Chill for a bit
                time.sleep(0.01)

class Streamer(threading.Thread):
    def __init__(self, host, port, frameQueue, loggingQueue):
        super(Streamer, self).__init__()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.socket.bind((host, port))
        self.frameQueue = frameQueue
        self.loggingQueue = loggingQueue
        self.start()

    def run(self):
        self.socket.listen()
        conn, addr = self.socket.accept()
        with conn:
            print('Streamer Connected')
            while True:
                startGetFrame = time.time()
                frame = self.frameQueue.get()
                data = cv.imencode('.jpg', frame)[1].tobytes()
                endGetFrame = time.time()

                startGetLength = time.time()
                length = pack('>Q', len(data))
                endGetLength = time.time()

                sendStart = time.time()
                conn.sendall(length)
                conn.sendall(data)
                sendEnd = time.time()

                startAck = time.time()
                ack = conn.recv(1)
                endAck = time.time()

                # Calculate times for tasks
                timeToGetFrame = (endGetFrame - startGetFrame) * 1000
                timeToGetLength = (endGetLength - startGetLength) * 1000
                timeToSend = (sendEnd - sendStart) * 1000
                timeForAck = (endAck - startAck) * 1000

                # Send the data to the logger
                loggingData = {
                    'id': 4, 
                    'timeToGetFrame': timeToGetFrame, 
                    'timeToGetLength': timeToGetLength, 
                    'timeToSend': timeToSend, 
                    'timeForAck': timeForAck
                }
                self.loggingQueue.put(loggingData)
                time.sleep(0.001)

class Logger(threading.Thread):
    def __init__(self, loggingQueue):
        super(Logger, self).__init__()
        self.loggingQueue = loggingQueue
        self.lastSaveTime = None
        self.savedData = True
        # Logs for the PipelineRunner
        self.grabPipelineTimesRunner = open('/media/loggingDrive/logs/PipelineRunner/PipelineRunner/grabPipelineTimes.csv', 'w+')
        self.setSensorTimes = open('/media/loggingDrive/logs/PipelineRunner/PipelineRunner/setSensorTimes.csv', 'w+')
        self.setLEDsTimes = open('/media/loggingDrive/logs/PipelineRunner/PipelineRunner/setLEDsTimes.csv', 'w+')

        # Logs for the Pipeline
        self.grabFrameTimes = open('/media/loggingDrive/logs/PipelineRunner/Pipeline/grabFrameTimes.csv', 'w+')
        self.grabPipelineTimes = open('/media/loggingDrive/logs/PipelineRunner/Pipeline/grabPipelineTimes.csv', 'w+')
        self.toHSVTimes = open('/media/loggingDrive/logs/PipelineRunner/Pipeline/toHSVTimes.csv', 'w+')
        self.threshTimes = open('/media/loggingDrive/logs/PipelineRunner/Pipeline/threshTimes.csv', 'w+')
        self.morphTimes = open('/media/loggingDrive/logs/PipelineRunner/Pipeline/morphTimes.csv', 'w+')
        self.findContoursTimes = open('/media/loggingDrive/logs/PipelineRunner/Pipeline/findContoursTimes.csv', 'w+')
        self.processContoursTimes = open('/media/loggingDrive/logs/PipelineRunner/Pipeline/processContoursTimes.csv', 'w+')
        self.filterContoursTimes = open('/media/loggingDrive/logs/PipelineRunner/Pipeline/filterContoursTimes.csv', 'w+')
        self.sortContoursTimes = open('/media/loggingDrive/logs/PipelineRunner/Pipeline/sortContoursTimes.csv', 'w+')
        self.pairContoursTimes = open('/media/loggingDrive/logs/PipelineRunner/Pipeline/pairContoursTimes.csv', 'w+')
        self.PnPTimes = open('/media/loggingDrive/logs/PipelineRunner/Pipeline/PnPTimes.csv', 'w+')
        self.drawTimes = open('/media/loggingDrive/logs/PipelineRunner/Pipeline/drawTimes.csv', 'w+')
        self.postResultsTimes = open('/media/loggingDrive/logs/PipelineRunner/Pipeline/postResultsTimes.csv', 'w+')
        self.postToStreamTimes = open('/media/loggingDrive/logs/PipelineRunner/Pipeline/postToStreamTimes.csv', 'w+')
        self.pipelineLatencies = open('/media/loggingDrive/logs/PipelineRunner/Pipeline/pipelineLatencies.csv', 'w+')

        # Logs for ProcessOutput
        self.getDataTimes = open('/media/loggingDrive/logs/PipelineRunner/ProcessOutput/getDataTimes.csv', 'w+')
        self.writeDataTimes = open('/media/loggingDrive/logs/PipelineRunner/ProcessOutput/writeDataTimes.csv', 'w+')
        self.identifierSwapTimes = open('/media/loggingDrive/logs/PipelineRunner/ProcessOutput/identifierSwapTimes.csv', 'w+')

        # Logs for IPCHandler
        self.sendResultsTimes = open('/media/loggingDrive/logs/PipelineRunner/IPCHandler/sendResultsTimes.csv', 'w+')
        self.resultsAckTimes = open('/media/loggingDrive/logs/PipelineRunner/IPCHandler/resultsAckTimes.csv', 'w+')
        self.recvPipelineTimes = open('/media/loggingDrive/logs/PipelineRunner/IPCHandler/recvPipelineTimes.csv', 'w+')
        self.recvRestartPipelineTimes = open('/media/loggingDrive/logs/PipelineRunner/IPCHandler/recvRestartPipelineTimes.csv', 'w+')
        self.decodeTimes = open('/media/loggingDrive/logs/PipelineRunner/IPCHandler/decodeTimes.csv', 'w+')

        # Logs for Streamer
        self.timesToGetFrame = open('/media/loggingDrive/logs/PipelineRunner/Streamer/timesToGetFrame.csv', 'w+')
        self.timesToGetLength = open('/media/loggingDrive/logs/PipelineRunner/Streamer/timesToGetLength.csv', 'w+')
        self.timesToSend = open('/media/loggingDrive/logs/PipelineRunner/Streamer/timesToSend.csv', 'w+')
        self.timesForAck = open('/media/loggingDrive/logs/PipelineRunner/Streamer/timesForAck.csv', 'w+')

        self.start()

    def run(self):
        while True:
            if self.savedData:
                self.lastSaveTime = time.time()
                self.savedData = False
            
            data = self.loggingQueue.get()

            if data['id'] == 0:
                self.grabPipelineTimesRunner.write('{0},'.format(data['grabPipelineTime']))
                self.setSensorTimes.write('{0},'.format(data['setSensorTime']))
                self.setLEDsTimes.write('{0},'.format(data['setLEDsTime']))
                
            elif data['id'] == 1:
                self.grabFrameTimes.write('{0},'.format(data['grabFrameTime']))
                self.grabPipelineTimes.write('{0},'.format(data['grabPipelineTime']))
                self.toHSVTimes.write('{0},'.format(data['toHSVTime']))
                self.threshTimes.write('{0},'.format(data['threshTime']))
                self.morphTimes.write('{0},'.format(data['morphTime']))
                self.findContoursTimes.write('{0},'.format(data['findContoursTime']))
                self.processContoursTimes.write('{0},'.format(data['processContoursTime']))
                self.filterContoursTimes.write('{0},'.format(data['filterContoursTime']))
                self.sortContoursTimes.write('{0},'.format(data['sortContoursTime']))
                self.pairContoursTimes.write('{0},'.format(data['pairContoursTime']))
                self.PnPTimes.write('{0},'.format(data['PnPTime']))
                self.drawTimes.write('{0},'.format(data['drawTime']))
                self.postResultsTimes.write('{0},'.format(data['postResultsTime']))
                self.postToStreamTimes.write('{0},'.format(data['postToStreamTime']))
                self.pipelineLatencies.write('{0},'.format(data['pipelineLatency']))

            elif data['id'] == 2:
                self.getDataTimes.write('{0},'.format(data['getDataTime']))
                self.writeDataTimes.write('{0},'.format(data['writeDataTime']))
                self.identifierSwapTimes.write('{0},'.format(data['identifierSwapTime']))

            elif data['id'] == 3:
                self.sendResultsTimes.write('{0},'.format(data['sendResultsTime']))
                self.resultsAckTimes.write('{0},'.format(data['resultsAckTime']))
                self.recvPipelineTimes.write('{0},'.format(data['recvPipelineTime']))
                self.recvRestartPipelineTimes.write('{0},'.format(data['recvRestartPipelineTime']))
                self.decodeTimes.write('{0},'.format(data['decodeTime']))

            elif data['id'] == 4:
                self.timesToGetFrame.write('{0},'.format(data['timeToGetFrame']))
                self.timesToGetLength.write('{0},'.format(data['timeToGetLength']))
                self.timesToSend.write('{0},'.format(data['timeToSend']))
                self.timesForAck.write('{0},'.format(data['timeForAck']))
            
            if (time.time() - self.lastSaveTime >= 5):
                self.grabPipelineTimesRunner.flush()
                os.fsync(self.grabPipelineTimesRunner.fileno())
                self.setSensorTimes.flush()
                os.fsync(self.setSensorTimes.fileno())
                self.setLEDsTimes.flush()
                os.fsync(self.setLEDsTimes.fileno())
                self.grabFrameTimes.flush()
                os.fsync(self.grabFrameTimes.fileno())
                self.grabPipelineTimes.flush()
                os.fsync(self.grabPipelineTimes.fileno())
                self.toHSVTimes.flush()
                os.fsync(self.toHSVTimes.fileno())
                self.threshTimes.flush()
                os.fsync(self.threshTimes.fileno())
                self.morphTimes.flush()
                os.fsync(self.morphTimes.fileno())
                self.findContoursTimes.flush()
                os.fsync(self.findContoursTimes.fileno())
                self.processContoursTimes.flush()
                os.fsync(self.processContoursTimes.fileno())
                self.filterContoursTimes.flush()
                os.fsync(self.filterContoursTimes.fileno())
                self.sortContoursTimes.flush()
                os.fsync(self.sortContoursTimes.fileno())
                self.pairContoursTimes.flush()
                os.fsync(self.pairContoursTimes.fileno())
                self.PnPTimes.flush()
                os.fsync(self.PnPTimes.fileno())
                self.drawTimes.flush()
                os.fsync(self.drawTimes.fileno())
                self.postResultsTimes.flush()
                os.fsync(self.postResultsTimes.fileno())
                self.postToStreamTimes.flush()
                os.fsync(self.postToStreamTimes.fileno())
                self.pipelineLatencies.flush()
                os.fsync(self.pipelineLatencies.fileno())
                self.getDataTimes.flush()
                os.fsync(self.getDataTimes.fileno())
                self.writeDataTimes.flush()
                os.fsync(self.writeDataTimes.fileno())
                self.identifierSwapTimes.flush()
                os.fsync(self.identifierSwapTimes.fileno())
                self.sendResultsTimes.flush()
                os.fsync(self.sendResultsTimes.fileno())
                self.resultsAckTimes.flush()
                os.fsync(self.resultsAckTimes.fileno())
                self.recvPipelineTimes.flush()
                os.fsync(self.recvPipelineTimes.fileno())
                self.recvRestartPipelineTimes.flush()
                os.fsync(self.recvRestartPipelineTimes.fileno())
                self.decodeTimes.flush()
                os.fsync(self.decodeTimes.fileno())
                self.timesToGetFrame.flush()
                os.fsync(self.timesToGetFrame.fileno())
                self.timesToGetLength.flush()
                os.fsync(self.timesToGetLength.fileno())
                self.timesToSend.flush()
                os.fsync(self.timesToSend.fileno())
                self.timesForAck.flush()
                os.fsync(self.timesForAck.fileno())
                self.savedData = True
            
            time.sleep(0.001)

if __name__ == '__main__':
    logger = Logger(loggingQueue)
    ipcHandler = IPCHandler(HOST, PORT, pipelineLock, resultsLock, resultsReady, restartPipeline, loggingQueue)
    streamer = Streamer(HOST, STREAMING_PORT, frameQueue, loggingQueue)
    pipelineRunner = PipelineRunner(frameQueue, pipelineLock, resultsLock, restartPipeline, resultsReady, loggingQueue)
    logger.join()