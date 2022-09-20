import torch
import cv2
from yolov5.utils.plots import Annotator, colors
from yolov5.models.common import Detections
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # or yolov5n - yolov5x6, custom
cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
stride, names, pt = model.stride, model.names, model.pt
while(True):
    ret, frame = cap.read()
    results : Detections = model(frame)
    for pred in results.pred:
        if pred.shape[0]:
            for *box, conf, cls in reversed(pred):  # xyxy, confidence, class
                label = f'{names[int(cls)]} {conf:.2f}'
                print(label)
        else:
            print("no detections")
    cv2.imshow('preview', results.render(True)[0])
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()