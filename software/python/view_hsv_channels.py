import cv2
import pyfakewebcam
import numpy as np

cap = cv2.VideoCapture(0)
width = 640
height = 480
fps = 15

cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, fps)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)

# Video sinks
camera = pyfakewebcam.FakeWebcam('/dev/video2', width, height)
camera2 = pyfakewebcam.FakeWebcam('/dev/video3', width, height)
camera3 = pyfakewebcam.FakeWebcam('/dev/video4', width, height)

def useVideo():
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Output each h s and v channel in each of the camera sinks
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h = np.zeros((height, width, 3), dtype=np.uint8)
        h[:, :, 0] = hsv[:, :, 0]
        h[:, :, 1] = hsv[:, :, 0]
        h[:, :, 2] = hsv[:, :, 0]
        camera.schedule_frame(h)
        s = np.zeros((height, width, 3), dtype=np.uint8)
        s[:, :, 0] = hsv[:, :, 1]
        s[:, :, 1] = hsv[:, :, 1]
        s[:, :, 2] = hsv[:, :, 1]
        camera2.schedule_frame(s)
        v = np.zeros((height, width, 3), dtype=np.uint8)
        v[:, :, 0] = hsv[:, :, 2]
        v[:, :, 1] = hsv[:, :, 2]
        v[:, :, 2] = hsv[:, :, 2]
        camera3.schedule_frame(v)

        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

useVideo()