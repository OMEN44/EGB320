import cv2

from robot.vision.pipeline import proccess

def useImage(self, image):
    img = cv2.imread(f'/home/pi/EGB320/software/python/test_data2/{image}.jpg')

    frames = proccess(self, img)

    for i in range(len(frames)):
        self.sink[i].schedule_frame(cv2.cvtColor(frames[i], cv2.COLOR_BGR2RGB))