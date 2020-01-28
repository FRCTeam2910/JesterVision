import time
from picamera import PiCamera
import numpy as np
import cv2 as cv

# Initialize the camera, define the resolution and framerate
cam = PiCamera(resolution=(320, 240), framerate=90)
cam.rotation = 180

# Wait for the sensor to warm up
time.sleep(2)

while True:
    frame = np.empty((240 * 320 * 3,), dtype=np.uint8)
    cam.capture(frame, format='rgb', use_video_port=True)
    frame = frame.reshape((240, 320, 3))

    cv.imshow('GetFrames', frame)
    if cv.waitKey(1) & 0xFF == ord('c'):
        cv.imwrite('{0}.jpg'.format(str(time.time())), frame)

    time.sleep(0.001)
