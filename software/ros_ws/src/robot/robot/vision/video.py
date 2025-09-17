import cv2
import numpy as np
import time

from std_msgs.msg import String

from robot.vision.pipeline import proccess

WIDTH = 640
HEIGHT = 480
FPS = 60

def initCalibration(self, dimensions=(WIDTH, HEIGHT), balance=1.0, dim2=None, dim3=None):
    self.magicNumbers = {
        'K': np.array([[471.6658227413098, 0.0, 332.03119818185894], [0.0, 470.8943767328793, 217.66233465552523], [0.0, 0.0, 1.0]]),
        'D': np.array([[-0.09769440410102902], [0.012961725037653245], [0.08903099552070662], [-0.13551749872814936]])
    }

    assert dimensions[0]/dimensions[1] == WIDTH/HEIGHT, "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if not dim2:
        dim2 = dimensions
    if not dim3:
        dim3 = dimensions
    scaled_K = self.magicNumbers['K'] * dimensions[0] / WIDTH  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, self.magicNumbers['D'], dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, self.magicNumbers['D'], np.eye(3), new_K, dim3, cv2.CV_16SC2)
    self.calibration = {
        'map1': map1,
        'map2': map2
    }


def undistort(self, frame):
    return cv2.remap(frame, self.calibration['map1'], self.calibration['map2'], interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

def useVideo(self, framePath=None):

    frame = None

    ret, frame = self.cap.read()
    if not ret:
        print("Failed to grab frame")
        return

    now = time.time()

    frame = undistort(self, frame)

    frames = proccess(self, frame)

    for i in range(len(frames)):
        # Draw the FPS in a box on the bottom left of the frame
        frames[i] = cv2.rectangle(frames[i], (5, HEIGHT - 30), (60, HEIGHT - 5), (0, 0, 0), -1)        
        frames[i] = cv2.putText(frames[i], f'FPS: {int(1/(time.time()-now))}', (14, HEIGHT - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1, cv2.LINE_AA)

        # Draw the pipeline on the top left of the frame
        frames[i] = cv2.rectangle(frames[i], (5, 35), (130, 50 + 20 * len(self.pipeline)), (0, 0, 0), -1)        
        frames[i] = cv2.putText(frames[i], 'Filters:', (14, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        for j in range(len(self.pipeline)):
            frames[i] = cv2.putText(frames[i], self.pipeline[j], (14, 65 + 20 * j), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        self.sink[i].schedule_frame(cv2.cvtColor(frames[i], cv2.COLOR_BGR2RGB))