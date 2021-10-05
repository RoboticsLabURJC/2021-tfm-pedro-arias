import numpy as np
import cv2
import yolo_utils

yolo4_tiny = yolo_utils.YOLOv4Tiny()
# yolo4 = yolo_utils.YOLOv4()

cap = cv2.VideoCapture("/dev/video0") # check this
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Flip verticallly
    frame = cv2.flip(frame, 0)

    # Our operations on the frame come here
    yolo4_tiny.detect_frame(frame)

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
