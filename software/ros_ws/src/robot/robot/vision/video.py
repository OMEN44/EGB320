import cv2
import numpy as np
import time

from std_msgs.msg import String

from robot.vision.pipeline import proccess

from robot.vision.utils import D, K, WIDTH, HEIGHT, OUTPUT_SCALE

def initCalibration(self, dimensions=(WIDTH, HEIGHT), balance=1.0, dim2=None, dim3=None):
    assert dimensions[0]/dimensions[1] == WIDTH/HEIGHT, "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if not dim2:
        dim2 = dimensions
    if not dim3:
        dim3 = dimensions
    scaled_K = K * dimensions[0] / WIDTH  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
    self.calibration = {
        'map1': map1,
        'map2': map2,
        'new_k': new_K
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
        # Resize image to 800x600
        frames[i] = cv2.resize(frames[i], (int(WIDTH / OUTPUT_SCALE), int(HEIGHT / OUTPUT_SCALE)))

        # Draw the FPS in a box on the bottom left of the frame
        frames[i] = cv2.rectangle(frames[i], (5, int(HEIGHT / OUTPUT_SCALE) - 30), (60, int(HEIGHT / OUTPUT_SCALE) - 5), (0, 0, 0), -1)        
        frames[i] = cv2.putText(frames[i], f'FPS: {int(1/(time.time()-now))}', (14, int(HEIGHT / OUTPUT_SCALE) - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1, cv2.LINE_AA)

        # Draw the pipeline on the top left of the frame
        frames[i] = cv2.rectangle(frames[i], (5, 35), (130, 50 + 20 * len(self.pipeline)), (0, 0, 0), -1)        
        frames[i] = cv2.putText(frames[i], 'Filters:', (14, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Display target item in bottom right
        if self.targetItem:
            frames[i] = cv2.rectangle(frames[i], (int(WIDTH / OUTPUT_SCALE) - 155, int(HEIGHT / OUTPUT_SCALE) - 30), (int(WIDTH / OUTPUT_SCALE) - 5, int(HEIGHT / OUTPUT_SCALE) - 5), (0, 0, 0), -1)
            frames[i] = cv2.putText(frames[i], f'Target: {self.targetItem}', (int(WIDTH / OUTPUT_SCALE) - 150, int(HEIGHT / OUTPUT_SCALE) - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        for j in range(len(self.pipeline)):
            frames[i] = cv2.putText(frames[i], self.pipeline[j], (14, 65 + 20 * j), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        self.sink[i].schedule_frame(cv2.cvtColor(frames[i], cv2.COLOR_BGR2RGB))
        # cv2.imshow(f'Output {i}', frames[i])