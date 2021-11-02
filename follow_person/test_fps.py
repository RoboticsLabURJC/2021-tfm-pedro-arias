import numpy as np
import cv2
import yolo_utils
import time

#yolo4_tiny = yolo_utils.YOLOv4Tiny()
yolo4 = yolo_utils.YOLOv4()

n_frames = 0
cap = cv2.VideoCapture("/dev/video0") # check this
while(True):
    if n_frames == 0:
        t0 = time.time()

    # Capture frame-by-frame
    ret, frame = cap.read()
    n_frames += 1

    # Flip verticallly
    # frame = cv2.flip(frame, 0)

    # Our operations on the frame come here
    yolo4.detect_frame(frame)

    # Display the resulting frame
    cv2.imshow('frame', frame)

    # Print FPS
    print(n_frames/(time.time() - t0))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
