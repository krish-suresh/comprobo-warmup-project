import rclpy
from rclpy.node import Node
import numpy as np
from enum import Enum
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import torch
from yolov5.utils.plots import Annotator, colors
from yolov5.models.common import Detections

Kp_angle = 0.8
CLOST_DIST = 0.4
OBS_DIST = 1.0
class State(Enum):
    NO_PERSON = 0
    CLOSE_TO_PERSON = 1
    OBSTACLE_DETECTED = 2
    FOLLOWING_PERSON = 3
    HAS_OBS = 4

class FiniteStateMachine(Node):
    def __init__(self):
        super().__init__('finite_state_machine_node')
        self.curr_state = State.NO_PERSON
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.camera_sub = self.create_subscription(Image, 'camera/image_raw', self.process_image, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.br = CvBridge()
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        self.names = self.model.names
        self.search_dir = 1
        self.person_loc = None
        self.close_ranges = []
        self.ranges = None
    def process_scan(self, msg:LaserScan):
        self.ranges = msg.ranges
        self.has_obs = len([r for r in msg.ranges[0:30]+msg.ranges[-30:] if r != 0.0 and r < OBS_DIST]) > 0

    def process_image(self, msg:Image):
        frame = self.br.imgmsg_to_cv2(msg)
        results : Detections = self.model(frame)
        for pred in results.pred:
            if pred.shape[0]:
                has_person = False
                for *box, conf, cls in reversed(pred):
                    if self.names[int(cls)] == "person" and conf > 0.5:
                        has_person = True
                        self.person_loc = [None, None]
                        self.person_loc[0] = ((box[0] + abs(box[0]-box[2])/2)-(frame.shape[1]/2))/(frame.shape[1]/2)
                        self.person_loc[1] = abs(box[0]-box[2])/frame.shape[1]
                    # label = f'{self.names[int(cls)]} {conf:.2f}'
                if not has_person:
                    self.person_loc = None
            else:
                self.person_loc = None
        im = cv2.cvtColor(results.render(True)[0], cv2.COLOR_BGR2RGB)
        cv2.imshow('preview', im)
        cv2.waitKey(1)

    def run_loop(self):
        print(self.curr_state)
        
        if self.curr_state == State.NO_PERSON:
            self.publisher.publish(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.8*self.search_dir)))
            if self.person_loc is not None:
                self.curr_state = State.FOLLOWING_PERSON
        elif self.curr_state == State.CLOSE_TO_PERSON:
            self.publisher.publish(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)))
            if self.person_loc is None:
                self.curr_state = State.NO_PERSON
            elif self.person_loc[1] < CLOST_DIST:
                self.curr_state = State.FOLLOWING_PERSON
                return
        elif self.curr_state == State.FOLLOWING_PERSON:
            if self.person_loc is None:
                self.curr_state = State.NO_PERSON
                return
            if self.person_loc[1] > CLOST_DIST:
                self.curr_state = State.CLOSE_TO_PERSON
                return
            if self.has_obs:
                self.curr_state = State.HAS_OBS
                return
            self.search_dir = 1 if self.person_loc[0] < 0 else -1
            turn_speed = -float(Kp_angle*self.person_loc[0])
            print(turn_speed)
            self.publisher.publish(Twist(linear=Vector3(x=0.2, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=turn_speed)))
        elif self.curr_state == State.HAS_OBS:
            self.publisher.publish(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)))

def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateMachine()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()