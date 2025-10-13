import cv2
import pyfakewebcam
import numpy as np

from robot_interfaces.msg import Poi

WIDTH = 640
HEIGHT = 480
OUTPUT_SCALE = 1.3
# Calculated using (isle marker width in px * distance to isle marker in cm) / real isle marker width in cm
FOCAL_LENGTH = (27 * 70) / 5.5 # undistorted frame
# FOCAL_LENGTH = (50 * 50) / 7 # undistorted frame
# FOCAL_LENGTH = (120 * 26) / 7 # distorted frame
FOV = 140

HISTORY_LEN = 15

K = np.array([[471.6658227413098, 0.0, 332.03119818185894], [0.0, 470.8943767328793, 217.66233465552523], [0.0, 0.0, 1.0]])
D = np.array([[-0.09769440410102902], [0.012961725037653245], [0.08903099552070662], [-0.13551749872814936]])

poi = None

def getEmptyPoi():
    global poi
    if poi is None:
        poi = Poi()
        poi.exists = False
        poi.distance = 0
        poi.bearing = [0, 0, 0]
    return poi

def getPoi(actualWidth, perceivedWidth, objectX, focalLength):
    poi = Poi()
    poi.exists = True
    poi.distance = int(objectDistanceWithCorrection(actualWidth, perceivedWidth, objectX, focalLength) * 1000)
    poi.bearing = [
        int(objectAngle(objectX) * 1000), 
        int(objectAngle(objectX + perceivedWidth / 2) * 1000), 
        int(objectAngle(objectX + perceivedWidth) * 1000)
    ]
    return poi

def objectDistance(actualWidth, perceivedWidth, focalLength):
    # range from 23 to 34px
    # actualWidth in cm
    # 100cm to 64cm


    # 63cm @ 63deg
    if perceivedWidth == 0:
        return 0
    # return np.round((actualWidth * FOCAL_LENGTH) / perceivedWidth, 2)
    return np.round((actualWidth * focalLength) / perceivedWidth, 2)

def objectDistanceWithCorrection(actualWidth, perceivedWidth, objectX, focalLength):
    angle = objectAngle(objectX) * (180 / np.pi)
    distance = objectDistance(actualWidth, perceivedWidth, focalLength)

    # Use quadratic approximation to correct distance based on angle
    return np.round((0.000143322 * (angle ** 2) + 1.03692) * distance, 2)


def objectAngle(objectX):
    # Calculate the angle of the object from the center of the frame
    centerX = WIDTH / 2
    dx = objectX - centerX
    angle = (dx / centerX) * (FOV / 2)
    return np.round(angle * (np.pi / 180), 2)

def setupFakeCam():
    camera1 = pyfakewebcam.FakeWebcam('/dev/video4', int(WIDTH / OUTPUT_SCALE), int(HEIGHT / OUTPUT_SCALE))
    camera2 = pyfakewebcam.FakeWebcam('/dev/video5', int(WIDTH / OUTPUT_SCALE), int(HEIGHT / OUTPUT_SCALE))
    camera3 = pyfakewebcam.FakeWebcam('/dev/video6', int(WIDTH / OUTPUT_SCALE), int(HEIGHT / OUTPUT_SCALE))

    print(f'Streaming at 1/{OUTPUT_SCALE} resolution ({int(WIDTH / OUTPUT_SCALE)}x{int(HEIGHT / OUTPUT_SCALE)}) to /dev/video4, /dev/video5, /dev/video6')

    return [camera1, camera2, camera3]

def setupCamera():
    cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    # cap.set(cv2.CAP_PROP_code)
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
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
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

    # Pretty print settings
    print(f'Camera settings:')
    print(f'  Auto White Balance: {"On" if auto_wb else "Off"}')
    print(f'  White Balance Temperature: {white_balance}')
    print(f'  Auto Exposure: {"On" if auto_exposure else "Off"}')
    print(f'  Exposure: {exposure}')
    print(f'  Brightness: {brightness}')
    print(f'  Contrast: {contrast}')
    print(f'  Saturation: {saturation}')
    print(f'  Hue: {hue}')
    print(f'  Gain: {gain}')
    print(f'  Gamma: {gamma}')
    print(f'  Sharpness: {sharpness}')
    print(f'  Backlight: {backlight}')
    print(f'  Temperature: {temperature}')
    print(f'Streaming at {WIDTH}x{HEIGHT} from /dev/video0')

    return cap

def filterFalsePositives(history):
    if len(history) == 0:
        return []
    
    pois = []
    
    
    for poiList in history:
        pass


    # print(counts, avg_count, len(pois))



    return pois

    