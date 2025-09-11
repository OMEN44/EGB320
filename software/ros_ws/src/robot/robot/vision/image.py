import cv2
import pyfakewebcam

from robot.vision.pipeline import proccess

WIDTH = 640
HEIGHT = 480

def useImage(self, image):
    img = cv2.imread(f'/home/pi/EGB320/software/python/test_data2/{image}.jpg')

    sink = setupFakeCam()

    frames = proccess(self, img)

    for i in range(len(frames)):
        sink[i].schedule_frame(cv2.cvtColor(frames[i], cv2.COLOR_BGR2RGB))



def setupFakeCam():
    camera1 = pyfakewebcam.FakeWebcam('/dev/video4', WIDTH, HEIGHT)
    camera2 = pyfakewebcam.FakeWebcam('/dev/video5', WIDTH, HEIGHT)
    camera3 = pyfakewebcam.FakeWebcam('/dev/video6', WIDTH, HEIGHT)

    return [camera1, camera2, camera3]