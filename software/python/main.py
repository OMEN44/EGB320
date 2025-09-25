import cv2

def set_res(cap, x,y):
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(x))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(y))
    return str(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),str(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

cap = cv2.VideoCapture(0)

res = set_res(cap, 1024, 768)
print(f"Resolution set to {res[0]}x{res[1]}")  # Print the resolution set
res = set_res(cap, 1280, 720)
print(f"Resolution set to {res[0]}x{res[1]}")

# Release the camera
cap.release()