import numpy as np
import cv2
import yolov4

dw = yolov4.DarknetWrapper(config="data/yolov4-tiny.cfg", data="data/coco.data", weights="data/yolov4-tiny.weights")
# dw = yolov4.DarknetWrapper()

cap = cv2.VideoCapture("/dev/video0") # check this
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Flip verticallly
    frame = cv2.flip(frame, 0)

    # Our operations on the frame come here
    dw.detect_frame(frame)

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()