import os
from flask import Flask, flash, redirect, url_for, render_template, request, Response, send_file, send_from_directory, safe_join, abort, jsonify
from flask_socketio import SocketIO, send, emit
import cv2 as cv
import numpy as np
import math
from queue import Queue
import threading
from camera_memory import Camera
import time
import random
import json
from werkzeug.utils import secure_filename
import io
from struct import pack, unpack
import socket
import copy

HOST = '127.0.0.1'
PORT = 5801
STREAMING_PORT = 5802

# Define some multithreading stuff
loggingQueue = Queue()
pipelineLock = threading.Lock()
resultsLock = threading.Lock()
savePipelinesEvent = threading.Event()
restartPipeline = threading.Event()
frameQueue = Queue(1)

def pipelinesToBytes(m_pipelines, activePipeline):
    for pipeline in m_pipelines:
        if (pipeline['objectPoints'] is not None):
            pipeline['objectPoints'] = pipeline['objectPoints'].tolist()

    data = {
        'activePipeline': activePipeline,
        'pipelines': m_pipelines
    }

    return json.dumps(data).encode('utf-8')

def pipelineToBytes(m_pipelines, activePipline):
    data = {
        'activePipeline': activePipeline,
        'vals': m_pipelines[activePipeline]
    }

    if (data['vals']['objectPoints'] is not None):
        data['vals']['objectPoints'] = data['vals']['objectPoints'].tolist()

    return json.dumps(data).encode('utf-8')

# Define some methods to load and store various types of info
def loadPipelines():
    pipelineData = json.loads(open('pipelines.json', 'r').readline())
    activePipeline = pipelineData['activePipeline']
    pipelines = pipelineData['pipelines']

    for pipeline in pipelines:
        if (pipeline['objectPoints'] is not None):
            pipeline['objectPoints'] = np.array(pipeline['objectPoints'], dtype=np.float32)

    for pipeline in pipelines:
        for val in pipeline:
            if (type(pipeline[val]) == list):
                pipeline[val] = tuple(pipeline[val])

    return activePipeline, pipelines

def loadNetworkingInfo():
    networkConfigFile = open('/media/loggingDrive/network_config.json', 'r')
    return json.loads(networkConfigFile.readline())

# Load up things stored on disk
activePipeline, pipelines = loadPipelines()
networkingInfo = loadNetworkingInfo()

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

class SavePipelines(threading.Thread):
    def __init__(self, pipelineLock, savePipelinesEvent):
        super(SavePipelines, self).__init__()
        self.pipelineLock = pipelineLock
        self.savePipelinesEvent = savePipelinesEvent
        self.start()

    def run(self):
        while True:
            # Wait for a change in the pipeline vals to save them
            self.savePipelinesEvent.wait()

            # Grab all the pipelines
            newPipelines = None
            newActivePipeline = None
            with self.pipelineLock:
                global pipelines
                global activePipeline
                newPipelines = copy.deepcopy(pipelines)
                newActivePipeline = activePipeline

            # Save the pipelines to the disk
            for pipeline in newPipelines:
                if (pipeline['objectPoints'] is not None):
                    pipeline['objectPoints'] = pipeline['objectPoints'].tolist()

            dataToSave = {
                'activePipeline': newActivePipeline,
                'pipelines': newPipelines
            }

            pipelinesFile = open('pipelines.json', 'w+')
            pipelinesFile.write(json.dumps(dataToSave))
            pipelinesFile.close()

            self.savePipelinesEvent.clear()

class IPCHandler(threading.Thread):
    def __init__(self, host, port, pipelineLock, resultsLock, restartPipeline, loggingQueue):
        super(IPCHandler, self).__init__()
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.pipelineLock = pipelineLock
        self.resultsLock = resultsLock
        self.restartPipeline = restartPipeline
        self.loggingQueue = loggingQueue
        self.start()

    def run(self):
        # Connect to the the other end
        self.socket.connect((self.host, self.port))
        print('IPC Handler Connected!')

        # Send over the active pipeline
        with self.pipelineLock:
            # global pipelines
            # global activePipeline

            data = pipelinesToBytes(copy.deepcopy(pipelines), activePipeline)
            length = pack('>Q', len(data))

            self.socket.sendall(length)
            self.socket.sendall(data)
        
        # Wait on the ack before proceeding
        ack = self.socket.recv(1)
        print('Got Ack!')

        while True:
            # Get results from the pipeline
            startGetResults = time.time()
            resultsBytes = self.socket.recv(2048)
            endGetResults = time.time()

            # Send back an ack
            startSendResultsAck = time.time()
            self.socket.sendall(b'\00')
            endSendResultsAck = time.time()

            # Send over the values of the acitve pipeline
            startSendPipeline = time.time()
            with self.pipelineLock:
                # global pipelines
                # global activePipeline

                self.socket.sendall(pipelineToBytes(copy.deepcopy(pipelines), activePipeline))
            endSendPipeline = time.time()

            # Wait on the ack
            startRecvPipelineAck = time.time()
            ack = self.socket.recv(1)
            endRecvPipelineAck = time.time()

            # Send over whether the pipeline needs to be restarted (new camera config)
            startSendRestartPipeline = time.time()
            if (self.restartPipeline.is_set()):
                self.socket.sendall(b'\01')
                self.restartPipeline.clear()
            else:
                self.socket.sendall(b'\00')
            endSendRestartPipeline = time.time()

            # Update our copy of the pipeline results
            startUpdateResults = time.time()
            with self.resultsLock:
                global results
                results = json.loads(resultsBytes.decode('utf-8'))
            endUpdateResults = time.time()

            # Calculate the times for each task
            getResultsTime = (endGetResults - startGetResults) * 1000
            sendResultsAckTime = (endSendResultsAck - startSendResultsAck) * 1000
            sendPipelineTime = (endSendPipeline - startSendPipeline) * 1000
            recvPipelineTime = (endRecvPipelineAck - startRecvPipelineAck) * 1000
            sendRestartPipelineTime = (endSendRestartPipeline - startSendRestartPipeline) * 1000
            updateResultsTime = (endUpdateResults - endUpdateResults) * 1000

            loggingData = {
                'id': 0,
                'getResultsTime': getResultsTime,
                'sendResultsAckTime': sendResultsAckTime,
                'sendPipelineTime': sendPipelineTime,
                'recvPipelineTime': recvPipelineTime,
                'sendRestartPipelineTime': sendRestartPipelineTime,
                'updateResultsTime': updateResultsTime
            }
            self.loggingQueue.put(loggingData)

            time.sleep(0.001)

class Streamer(threading.Thread):
    def __init__(self, host, port, frameQueue, loggingQueue):
        super(Streamer, self).__init__()
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.frameQueue = frameQueue
        self.loggingQueue = loggingQueue
        self.start()

    def run(self):
        # Connect to the streamer
        self.socket.connect((self.host, self.port))
        print('Streamer Connected!')

        while True:
            # Get the length of image and the image itself
            recvStart = time.time()
            lengthInBytes = self.socket.recv(8)
            (length,) = unpack('>Q', lengthInBytes)
            imgData = b''
            while len(imgData) < length:
                bytesLeftToRead = length - len(imgData)
                imgData+=self.socket.recv(4096 if bytesLeftToRead > 4096 else bytesLeftToRead)
            recvEnd = time.time()

            # Send back an acknowledgement that we received the data
            ackStart = time.time()
            assert len(b'\00') == 1
            self.socket.sendall(b'\00')
            ackEnd = time.time()

            # Put it through the queue, might as well time it for now
            processStart = time.time()
            if (not self.frameQueue.full()):
                self.frameQueue.put(imgData)
            processEnd = time.time()

            # Calculate the times for each task, pass it to the logger
            recvTime = (recvEnd - recvStart) * 1000
            ackTime = (ackEnd - ackStart) * 1000
            proccessTime = (processEnd - processStart) * 1000

            loggingData = {
                'id': 1, 
                'recvTime': recvTime,
                'ackTime': ackTime, 
                'processTime': proccessTime
            }
            self.loggingQueue.put(loggingData)
            
            # Chill out for a bit
            time.sleep(0.001)

class Logger(threading.Thread):
    def __init__(self, loggingQueue):
        super(Logger, self).__init__()
        self.loggingQueue = loggingQueue
        self.lastSaveTime = None
        self.savedData = True
        # Logs for the IPCHandler
        self.getResultsTimes = open('/media/loggingDrive/logs/init/IPCHandler/getResultsTimes.csv', 'w+')
        self.sendResultsAckTimes = open('/media/loggingDrive/logs/init/IPCHandler/sendResultsAckTimes.csv', 'w+')
        self.sendPipelineTimes = open('/media/loggingDrive/logs/init/IPCHandler/sendPipelineTimes.csv', 'w+')
        self.recvPipelineTimes = open('/media/loggingDrive/logs/init/IPCHandler/recvPipelineTimes.csv', 'w+')
        self.sendRestartPipelineTimes = open('/media/loggingDrive/logs/init/IPCHandler/sendRestartPipelineTimes.csv', 'w+')
        self.updateResultsTimes = open('/media/loggingDrive/logs/init/IPCHandler/updateResultsTimes.csv', 'w+')

        # These are logs for Streamer
        self.recvTimes = open('/media/loggingDrive/logs/init/Streamer/recvTimes.csv', 'w+')
        self.ackTimes = open('/media/loggingDrive/logs/init/Streamer/ackTimes.csv', 'w+')
        self.processTimes = open('/media/loggingDrive/logs/init/Streamer/processTimes.csv', 'w+')

        self.start()

    def run(self):
        while True:
            if self.savedData:
                self.lastSaveTime = time.time()
                self.savedData = False
            
            data = self.loggingQueue.get()

            if data['id'] == 0:
                self.getResultsTimes.write('{0},'.format(data['getResultsTime']))
                self.sendResultsAckTimes.write('{0},'.format(data['sendResultsAckTime']))
                self.sendPipelineTimes.write('{0},'.format(data['sendPipelineTime']))
                self.recvPipelineTimes.write('{0},'.format(data['recvPipelineTime']))
                self.sendRestartPipelineTimes.write('{0},'.format(data['sendRestartPipelineTime']))
                self.updateResultsTimes.write('{0},'.format(data['updateResultsTime']))
            elif data['id'] == 1:
                self.recvTimes.write('{0},'.format(data['recvTime']))
                self.ackTimes.write('{0},'.format(data['ackTime']))
                self.processTimes.write('{0},'.format(data['processTime']))
                
            
            if (time.time() - self.lastSaveTime >= 1):
                self.getResultsTimes.flush()
                os.fsync(self.getResultsTimes.fileno())
                self.sendResultsAckTimes.flush()
                os.fsync(self.sendResultsAckTimes.fileno())
                self.sendPipelineTimes.flush()
                os.fsync(self.sendPipelineTimes.fileno())
                self.recvPipelineTimes.flush()
                os.fsync(self.recvPipelineTimes.fileno())
                self.sendRestartPipelineTimes.flush()
                os.fsync(self.sendRestartPipelineTimes.fileno())
                self.updateResultsTimes.flush()
                os.fsync(self.updateResultsTimes.fileno())
                self.recvTimes.flush()
                os.fsync(self.recvTimes.fileno())
                self.ackTimes.flush()
                os.fsync(self.ackTimes.fileno())
                self.processTimes.flush()
                os.fsync(self.processTimes.fileno())
                self.savedData = True
            
            time.sleep(0.001)

# This method begins the background threads and returns the Flask application instance to the caller
def createApp():
    logger = Logger(loggingQueue)
    savePipelines = SavePipelines(pipelineLock, savePipelinesEvent)
    ipcHandler = IPCHandler(HOST, PORT, pipelineLock, resultsLock, restartPipeline, loggingQueue)
    streamer = Streamer(HOST, STREAMING_PORT, frameQueue, loggingQueue)
    return Flask(__name__)

# Now we create the app and the SocketIO instance
app = createApp()
app.secret_key = 'test'
app.config['UPLOAD_FOLDER'] = '/media/loggingDrive'
socketio = SocketIO(app)

# Regular request handlers for the server
@app.route('/')
def index():
    return render_template('index.html', data=networkingInfo)

def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen(Camera(frameQueue)),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

ALLOWED_EXTENSIONS = set(['json'])

def allowed_file(filename):
    return '.' in filename and \
           filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

@app.route('/Pipeline', methods=['GET'])
def handlePipelineRequest():
    global pipelines
    global activePipeline
    pipeline = pipelines[activePipeline].copy()
    if (pipeline['objectPoints'] is not None):
        pipeline['objectPoints'] = pipeline['objectPoints'].tolist()
    return jsonify(pipelines[activePipeline])

@app.route('/CameraConfig', methods=['POST'])
def handle_camera_config_file():
    if 'file' not in request.files:
        flash('No file part')
        return redirect(request.url)
    file = request.files['file']
    # if user does not select file, browser also
    # submit an empty part without filename
    if file.filename == '':
        flash('No selected file')
        return redirect(request.url)
    if file and allowed_file(file.filename):
        filename = secure_filename(file.filename)
        filename = 'cameraConfig.json'
        file.save(os.path.join(app.config['UPLOAD_FOLDER'], filename))
        restartPipeline.set()
        return redirect(url_for('index'))

# Request Handlers for the websocket
def sendPipelines(m_pipelines, activePipeline):
    dataToSend = pipelinesToBytes(m_pipelines, activePipeline).decode('utf-8')
    emit('initData', dataToSend)
    print('sent pipelines!')

# event handler for when a client connects
@socketio.on('connect', namespace='/ws')
def onConnect():
    print('client connected!')
    # Send the pipelines
    global pipelines
    global activePipeline
    sendPipelines(copy.deepcopy(pipelines), activePipeline)

# event handler for updating existing pipelines and adding new ones
@socketio.on('updatePipelineVals', namespace='/ws')
def updatePipeline(data):
    data = json.loads(data)
    newPipelineVals = data['vals']

    # Check if we have new pipeline vals that need to be saved to the disk
    # Copy the vals we have stored to see if there are any differences
    global pipelines
    global activePipeline
    currentVals = pipelines[activePipeline].copy()

    # Check if there are any object points to convert to a list
    if (currentVals['objectPoints'] is not None):
        currentVals['objectPoints'] = currentVals['objectPoints'].tolist()

    for val in currentVals:
        if (type(currentVals[val]) == tuple):
            currentVals[val] = list(currentVals[val])

    # Now we can compare to see if we have new values
    if not (currentVals == newPipelineVals):
        # If they are different, trigger the update event to save the new values to the disk
        savePipelinesEvent.set()

    # Now, we update the gobal copy of the pipeline
    # Convert the object points to a numpy array if we need to
    if (newPipelineVals['objectPoints'] is not None):
        newPipelineVals['objectPoints'] = np.array(newPipelineVals['objectPoints'], dtype=np.float32)

    # Convert all lists to tuples
    for val in newPipelineVals:
        if (type(newPipelineVals[val]) == list):
            newPipelineVals[val] = tuple(newPipelineVals[val])

    # Acquire the lock, update our pipelines
    global pipelineLock
    with pipelineLock:
        activePipeline = data['activePipeline']
        pipelines[activePipeline] = newPipelineVals

    # Send the pipeline results back to the client
    with resultsLock:
        emit('pipelineResults', json.dumps(results))

# event handler for sending results (normally used by the robot)
@socketio.on('getPipelineResults', namespace='/ws')
def sendResults():
    with resultsLock:
        emit('pipelineResults', json.dumps(results))

# event handler for updating networking values
@socketio.on('updateNetworkingValues', namespace='/ws')
def updateNetworkingValues(newNetworkValues):
    print('recieved updated network values!')
    networkingConfigFile = open('/media/loggingDrive/network_config.json', 'w+')
    networkingConfigFile.write(newNetworkValues)
    networkingConfigFile.close()

# event handler for when a client disconnects
@socketio.on('disconnect', namespace='/ws')
def onDisconnect():
    print('client disconnected!')

if __name__ == '__main__':
    socketio.run(app, debug=False)