import cv2
import pyfakewebcam
import numpy as np
import time

# import robot.vision.isle_marker as isle_marker

WIDTH = 640
HEIGHT = 480
FPS = 15

def useVideo():
    sink = setupFakeCam()
    cap = setupCamera()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        now = time.time()

        # isle_marker.findMarkers(frame)
        cv2.GaussianBlur(frame, (7, 7), 0, frame)

        frame = cv2.putText(frame, f"FPS: {1/(time.time()-now):.2f}", (10, HEIGHT - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        sink[0].schedule_frame(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        sink[1].schedule_frame(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        sink[2].schedule_frame(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))


def setupCamera():
    cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    # Disable auto exposure
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, 10)
    # Disable auto white balance
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    # cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4600)

    return cap

def setupFakeCam():
    camera1 = pyfakewebcam.FakeWebcam('/dev/video4', WIDTH, HEIGHT)
    camera2 = pyfakewebcam.FakeWebcam('/dev/video5', WIDTH, HEIGHT)
    camera3 = pyfakewebcam.FakeWebcam('/dev/video6', WIDTH, HEIGHT)

    return [camera1, camera2, camera3]