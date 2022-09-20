import rclpy
from rclpy.node import Node
import numpy as np
from enum import Enum
from geometry_msgs.msg import Twist, Vector3, Quaternion
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import torch
from yolov5.utils.plots import Annotator, colors
from yolov5.models.common import Detections
import math

Kp_angle = 0.8
CLOST_DIST = 0.4
OBS_DIST = 1.0

FORCE_PARAMETER_Y = -40 
FORCE_PARAMETER_X = -40
ATTRACTIVE_PARAMETER = 5000
LIN_VEL_SCALE = 15
ANG_VEL_SCALE = 50
AVOIDED_DISTANCE = 1.0

def euler_from_quaternion(quat: Quaternion):
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return yaw_z

class State(Enum):
    NO_PERSON = 0
    CLOSE_TO_PERSON = 1
    OBSTACLE_DETECTED = 2
    FOLLOWING_PERSON = 3
    HAS_OBS = 4

class ObstacleDetection():
    def __init__(self):
        self.destination = np.array([0, 0])
        self.obstacle_avoided = True
        self.curr_pos = 0.0
        self.curr_heading = [0.0, 0.0]
        self.ranges = None

    @property
    def dist_from_target(self):
        return np.linalg.norm(self.destination - self.curr_pos)

    @property
    def angular_diff(self):
        return math.degrees(np.arcsin(np.cross(self.curr_heading, self.heading)))

    def update_destination(self):
        self.destination = self.curr_pos + AVOIDED_DISTANCE*self.curr_heading

    def calculate_net_force(self):
        self.attractive_forces = ATTRACTIVE_PARAMETER/self.dist_from_target*(self.destination - self.curr_pos)
        forces = []
        for angle, point in enumerate(self.ranges):
            if point > 0.0:
                forces.append([FORCE_PARAMETER_X*math.cos(math.radians(angle))/point**2, FORCE_PARAMETER_Y*math.sin(math.radians(angle))/point**2])
        self.repellent_forces = np.sum(np.array(forces), axis=0)
        self.net_force = self.attractive_forces + self.repellent_forces

    def calculate_heading(self):
        self.calculate_net_force()
        if np.linalg.norm(self.net_force) == 0:
            self.heading = np.array([0, 0])
        else:
            self.heading = self.net_force/np.linalg.norm(self.net_force)

    def calculate_vel(self):
        self.calculate_heading()
        if self.dist_from_target < 0.05:
            self.obstacle_avoided = True
        return Twist(linear=Vector3(x=0.15, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=self.angular_diff/ANG_VEL_SCALE))


class FiniteStateMachine(Node):
    def __init__(self):
        super().__init__('finite_state_machine_node')
        self.curr_state = State.NO_PERSON
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.camera_sub = self.create_subscription(Image, 'camera/image_raw', self.process_image, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.update_pose, 10)
        self.br = CvBridge()
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        self.names = self.model.names
        self.search_dir = 1
        self.person_loc = None
        self.close_ranges = []
        self.ranges = None
        self.obstacle_handler = ObstacleDetection()

    def update_pose(self, msg: Odometry):
        self.obstacle_handler.curr_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.orientation = euler_from_quaternion(msg.pose.pose.orientation)
        self.obstacle_handler.curr_heading = np.array([math.cos(self.orientation), math.sin(self.orientation)])

    def process_scan(self, msg:LaserScan):
        self.ranges = msg.ranges
        self.obstacle_handler.ranges = msg.ranges
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
                self.obstacle_handler.obstacle_avoided = False
                self.obstacle_handler.update_destination()
                return
            self.search_dir = 1 if self.person_loc[0] < 0 else -1
            turn_speed = -float(Kp_angle*self.person_loc[0])
            print(turn_speed)
            self.publisher.publish(Twist(linear=Vector3(x=0.2, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=turn_speed)))
        elif self.curr_state == State.HAS_OBS:
            if self.obstacle_handler.obstacle_avoided == True:
                if self.person_loc is not None:
                    self.curr_state = State.FOLLOWING_PERSON
                else:
                    self.curr_state = State.NO_PERSON
                return
            self.publisher.publish(self.obstacle_handler.calculate_vel())

def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateMachine()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()