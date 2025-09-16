import cv2
import pyfakewebcam
import numpy as np

WIDTH = 640
HEIGHT = 480
# Calculated using (isle marker width in px * distance to isle marker in cm) / real isle marker width in cm
FOCAL_LENGTH = (32 * 100) / 7
FOV = 140

def objectDistance(actualWidth, perceivedWidth):
    if perceivedWidth == 0:
        return 0
    return np.round((actualWidth * FOCAL_LENGTH) / perceivedWidth, 2)

def objectAngle(objectX):
    # Calculate the angle of the object from the center of the frame
    centerX = WIDTH / 2
    dx = objectX - centerX
    angle = (dx / centerX) * (FOV / 2)
    return np.round(angle, 2)

def setupFakeCam():
    camera1 = pyfakewebcam.FakeWebcam('/dev/video4', WIDTH, HEIGHT)
    camera2 = pyfakewebcam.FakeWebcam('/dev/video5', WIDTH, HEIGHT)
    camera3 = pyfakewebcam.FakeWebcam('/dev/video6', WIDTH, HEIGHT)

    return [camera1, camera2, camera3]

def setupCamera():
    cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    # cap.set(cv2.CAP_PROP_FPS, FPS)

    # Disable auto exposure
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, 5)
    # Disable auto white balance
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4600)

    cap.set(cv2.CAP_PROP_BRIGHTNESS, -0)
    cap.set(cv2.CAP_PROP_CONTRAST, 3)

    return cap

# takes arguments for each camera setting all with default values
def setupCameraWithDefaults(auto_wb=True, auto_exposure=True, brightness=0, contrast=3, saturation=56, hue=0, white_balance=4600, gamma=84, gain=1, temperature=4600, sharpness=2, backlight=0, exposure=156):
    cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
    cap.set(cv2.CAP_PROP_CONTRAST, contrast)
    cap.set(cv2.CAP_PROP_SATURATION, saturation) # 69
    cap.set(cv2.CAP_PROP_HUE, hue)
    cap.set(cv2.CAP_PROP_IOS_DEVICE_WHITEBALANCE, white_balance)
    cap.set(cv2.CAP_PROP_AUTO_WB, 1 if auto_wb else 0)
    cap.set(cv2.CAP_PROP_GAMMA, gamma)
    cap.set(cv2.CAP_PROP_GAIN, gain)
    cap.set(cv2.CAP_PROP_WB_TEMPERATURE, white_balance)
    cap.set(cv2.CAP_PROP_TEMPERATURE, temperature)
    cap.set(cv2.CAP_PROP_SHARPNESS, sharpness)
    cap.set(cv2.CAP_PROP_BACKLIGHT, backlight) # 121
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3 if auto_exposure else 1) # 3
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure) # 10

    return cap