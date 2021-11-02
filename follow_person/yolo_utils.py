import sys
sys.path.append("/home/parias/repos/darknet")

import cv2
import numpy as np
import darknet


class DarknetWrapper:
    def __init__(self, config, data, weights):
        self.network, self.class_names, self.class_colors = darknet.load_network(config, data, weights)

        self.width = darknet.network_width(self.network)
        self.height = darknet.network_height(self.network)

    # darknet helper function to run detection on image
    def run_detector(self, img, width=None, height=None):
        if width is None:
            width = self.width
        if height is None:
            height = self.height
        darknet_image = darknet.make_image(width, height, 3)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img_rgb, (width, height), interpolation=cv2.INTER_LINEAR)

        # get image ratios to convert bounding boxes to proper size
        img_height, img_width, _ = img.shape
        width_ratio = img_width/width
        height_ratio = img_height/height

        # run model on darknet style image to get detections
        darknet.copy_image_from_bytes(darknet_image, img_resized.tobytes())
        detections = darknet.detect_image(self.network, self.class_names, darknet_image)
        darknet.free_image(darknet_image)
        return detections, width_ratio, height_ratio

    def detect_image(self, img_path):
        image = cv2.imread(img_path)
        detections, width_ratio, height_ratio = self.run_detector(image)

        for label, confidence, bbox in detections:
            left, top, right, bottom = darknet.bbox2points(bbox)
            left, top, right, bottom = int(left * width_ratio), int(top * height_ratio), int(right * width_ratio), int(bottom * height_ratio)
            cv2.rectangle(image, (left, top), (right, bottom), self.class_colors[label], 2)
            cv2.putText(image, "{} [{:.2f}]".format(label, float(confidence)),
                        (left, top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        self.class_colors[label], 2)
        cv2.imshow("Detection", image)
        cv2.waitKey(0)

    def detect_frame(self, frame):
        # call our darknet helper on video frame
        detections, width_ratio, height_ratio = self.run_detector(frame)

        # loop through detections and draw them on transparent overlay image
        for label, confidence, bbox in detections:
          left, top, right, bottom = darknet.bbox2points(bbox)
          left, top, right, bottom = int(left * width_ratio), int(top * height_ratio), int(right * width_ratio), int(bottom * height_ratio)
          cv2.rectangle(frame, (left, top), (right, bottom), self.class_colors[label], 2)
          cv2.putText(frame, "{} [{:.2f}]".format(label, float(confidence)),
                            (left, top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            self.class_colors[label], 2)

    def detect_object(self, frame, object_):
        # call our darknet helper on video frame
        height, width, channels = frame.shape
        detections, width_ratio, height_ratio = self.run_detector(frame, width, height)

        # loop through detections and return first object match
        for label, confidence, bbox in detections:
            if label == object_:
                left, top, right, bottom = darknet.bbox2points(bbox)
                left, top, right, bottom = int(left * width_ratio), int(top * height_ratio), int(right * width_ratio), int(bottom * height_ratio)
                return label, confidence, (left, top, right, bottom)
        return None, 0, (0, 0, 0, 0)

class YOLOv4(DarknetWrapper):
    CONFIG_FILE = "data/yolov4.cfg"
    DATA_FILE = "data/coco.data"
    WEIGHTS_FILE = "data/yolov4.weights"

    def __init__(self):
        DarknetWrapper.__init__(self, config=self.CONFIG_FILE, 
            data=self.DATA_FILE, weights=self.WEIGHTS_FILE)


class YOLOv4Tiny(DarknetWrapper):
    CONFIG_FILE = "data/yolov4-tiny.cfg"
    DATA_FILE = "data/coco.data"
    WEIGHTS_FILE = "data/yolov4-tiny.weights"

    def __init__(self):
        DarknetWrapper.__init__(self, config=self.CONFIG_FILE, 
            data=self.DATA_FILE, weights=self.WEIGHTS_FILE)


if __name__ == "__main__":
    yolo = YOLOv4()
    yolo.detect_image("data/person.jpg")
