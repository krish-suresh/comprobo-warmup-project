import torch
import cv2
from yolov5.utils.plots import Annotator, colors
from yolov5.models.common import Detections

# Load yolov5 model and pretrained weights
model = torch.hub.load("ultralytics/yolov5", "yolov5s")
# Set up webcam image capture
cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
names = model.names  # Load list of names from model
while True:
    # Get current frame from webcam
    ret, frame = cap.read()
    # Process frame into list of detections
    results: Detections = model(frame)
    # Parse and output detections by class and confidence
    for pred in results.pred:
        if pred.shape[0]:
            for *box, conf, cls in reversed(pred):  # xyxy, confidence, class
                label = f"{names[int(cls)]} {conf:.2f}"
                print(label)
        else:
            print("no detections")
    # Display bounding boxes around detected objects
    cv2.imshow("preview", results.render(True)[0])
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
