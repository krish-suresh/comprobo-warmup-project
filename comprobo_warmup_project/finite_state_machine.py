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
from yolov5.models.common import Detections
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
import math
from geometry_msgs.msg import Pose, Vector3, Point
from builtin_interfaces.msg import Duration

Kp_angle = 0.8  # Proportional constant to orient towards person
CLOSE_DIST = 0.4  # (proportion of image) 
OBS_DIST = 1.0  # (m) min dist for obs avoidance to be triggered
FORWARD_SPEED = 0.2  # (m/s) constant following speed
OBS_DET_ANGLE = 30 # (degrees) half of scan zone for detection obs in front of neato

# Constants for the obstacle avoidance workflow.
FORCE_PARAMETER_Y = -50
FORCE_PARAMETER_X = -50
ATTRACTIVE_PARAMETER = 3000
LIN_VEL_SCALE = 15
ANG_VEL_SCALE = 50
AVOIDED_DISTANCE = 2.0  # (m) target distance in front of neato to avoid obs


def euler_from_quaternion(quat: Quaternion):
    """
    This is a helper function that converts quaternions (used to determine the
    orientation of the Neato) to roll, pitch, and yaw, which can more directly
    be analyzed to determine a vector for the heading of the Neato.
    """
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
    """
    This is an Enum used to manage the different states that the Neato can be
    in during execution of the finite state machine.
    """

    # No person detected in front of the Neato
    NO_PERSON = 0
    # Neato is within a given threshold distance of the person
    CLOSE_TO_PERSON = 1
    # There is an obstacle that needs to be avoided as the Neato is following a
    # person.
    OBSTACLE_DETECTED = 2
    # The Neato is following a detected person
    FOLLOWING_PERSON = 3


class ObstacleDetection:
    """
    This class is implemented to abstract the obstacle avoidance code away from
    the actual finite state machine Node. This handler determines how the Neato
    must avoid a given object and determines when the obstacle has been
    sufficiently avoided.
    """

    def __init__(self):
        self.destination = np.array([0, 0])
        self.obstacle_avoided = True
        self.curr_pos = 0.0
        self.curr_heading = [0.0, 0.0]
        self.ranges = None

    @property
    def dist_from_target(self):
        """
        Determines the distance left to travel to avoid the obstacle.
        """
        return np.linalg.norm(self.destination - self.curr_pos)

    @property
    def angular_diff(self):
        """
        Determines the angular difference between the Neato's heading and the
        intended direction of motion.
        """
        return math.degrees(np.arcsin(np.cross(self.curr_heading, self.heading)))

    def update_destination(self):
        """
        Updates the destination (position that is defined to have successfully
        avoided the obstacle).
        """
        self.destination = self.curr_pos + AVOIDED_DISTANCE * self.curr_heading

    def calculate_net_force(self):
        """
        Determines the net force on the Neato, which is used to publish
        velocities.
        """
        self.attractive_forces = (
            ATTRACTIVE_PARAMETER
            / self.dist_from_target
            * (self.destination - self.curr_pos)
        )
        forces = []
        for angle, point in enumerate(self.ranges):
            if point > 0.0:
                forces.append(
                    [
                        FORCE_PARAMETER_X *
                        math.cos(math.radians(angle)) / point**2,
                        FORCE_PARAMETER_Y *
                        math.sin(math.radians(angle)) / point**2,
                    ]
                )
        self.repellent_forces = np.sum(np.array(forces), axis=0)
        self.net_force = self.attractive_forces + self.repellent_forces

    def calculate_heading(self):
        """
        Updates the necessary heading that the Neato must travel based on the
        calculated net force.
        """
        self.calculate_net_force()
        if np.linalg.norm(self.net_force) == 0:
            self.heading = np.array([0, 0])
        else:
            self.heading = self.net_force / np.linalg.norm(self.net_force)

    def calculate_vel(self):
        """
        Calculates the velocity that the Neato needs to travel based on the
        current net force in order to avoid an obstacle.
        """
        self.calculate_heading()
        if self.dist_from_target < 0.2:
            self.obstacle_avoided = True
        return Twist(
            linear=Vector3(x=0.1, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=self.angular_diff / ANG_VEL_SCALE),
        )


class FiniteStateMachine(Node):
    def __init__(self):
        super().__init__("finite_state_machine_node")
        self.curr_state = State.NO_PERSON
        timer_period = 0.1
        # Main control loop
        self.timer = self.create_timer(timer_period, self.run_loop)

        # Neato pub/sub
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.camera_sub = self.create_subscription(
            Image, "camera/image_raw", self.process_image, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.process_scan, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.update_pose, 10)
        self.marker_pub = self.create_publisher(Marker, "test_marker", 10)

        self.br = CvBridge()  # used to process ROS images to opencv
        # YOLO model to detect objects
        self.model = torch.hub.load("ultralytics/yolov5", "yolov5s")
        self.names = self.model.names  # list of names from model
        self.search_dir = 1  # direction used to track person when they move out of view
        # information about person's location when detection exists in camera frame
        self.person_loc = None
        self.ranges = None  # Lidar data
        self.obstacle_handler = ObstacleDetection()  # container for obs avoidance logic

    def update_pose(self, msg: Odometry):
        """Callback to subscribe to the /odom topic and update position for obs avoidance"""
        self.obstacle_handler.curr_pos = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y]
        )
        self.orientation = euler_from_quaternion(msg.pose.pose.orientation)
        self.obstacle_handler.curr_heading = np.array(
            [math.cos(self.orientation), math.sin(self.orientation)]
        )

    def process_scan(self, msg: LaserScan):
        """Read lidar scan and check if an obstacle exists in front of the robot"""
        self.ranges = msg.ranges
        self.obstacle_handler.ranges = msg.ranges
        self.has_obs = (
            len(
                [
                    r
                    for r in msg.ranges[0:OBS_DET_ANGLE] + msg.ranges[-OBS_DET_ANGLE:]
                    if r != 0.0 and r < OBS_DIST
                ]
            )
            > 0
        )

    def process_image(self, msg: Image):
        """Process raw image through YOLO obj detection and update person_loc"""
        frame = self.br.imgmsg_to_cv2(msg) # convert ROSImage to opencv
        results: Detections = self.model(frame) # Run image through YOLO
        for pred in results.pred:
            if pred.shape[0]:
                has_person = False
                for *box, conf, cls in reversed(pred):
                    # When we are conf a person is visable update person_loc
                    if self.names[int(cls)] == "person" and conf > 0.5:
                        has_person = True
                        self.person_loc = [None, None]
                        # Compute location of person as percentage from middle of image [-1,1]
                        # and pseudo distance as width of bounding box / width of image
                        self.person_loc[0] = (
                            (box[0] + abs(box[0] - box[2]) / 2) -
                            (frame.shape[1] / 2)
                        ) / (frame.shape[1] / 2)
                        self.person_loc[1] = abs(
                            box[0] - box[2]) / frame.shape[1]
                    # label = f'{self.names[int(cls)]} {conf:.2f}'
                if not has_person:
                    self.person_loc = None
            else:
                self.person_loc = None
        # Display bounding box image
        im = cv2.cvtColor(results.render(True)[0], cv2.COLOR_BGR2RGB)
        cv2.imshow("preview", im)
        cv2.waitKey(1)

    def run_loop(self):
        """Main control loop"""
        print(self.curr_state)

        if self.curr_state == State.NO_PERSON:  # State for when current person location is unknown
            self.cmd_vel_pub.publish(
                Twist(
                    linear=Vector3(x=0.0, y=0.0, z=0.0),
                    # Turn in direction person was last seen
                    angular=Vector3(x=0.0, y=0.0, z=0.8 * self.search_dir),
                )
            )
            if self.person_loc is not None:  # Switch to following when person is found
                self.curr_state = State.FOLLOWING_PERSON
        elif self.curr_state == State.CLOSE_TO_PERSON:  # Stop when person is close to Neato
            self.cmd_vel_pub.publish(
                Twist(
                    linear=Vector3(x=0.0, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=0.0),
                )
            )
            # Switch to person not found or following based on current person location
            if self.person_loc is None:
                self.curr_state = State.NO_PERSON
            elif self.person_loc[1] < CLOSE_DIST:
                self.curr_state = State.FOLLOWING_PERSON
                return
        elif self.curr_state == State.FOLLOWING_PERSON:  # Control state where we orient and drive to person
            if self.person_loc is None:
                self.curr_state = State.NO_PERSON
                return
            if self.person_loc[1] > CLOSE_DIST:
                self.curr_state = State.CLOSE_TO_PERSON
                return
            # switch states if we detect an obstacle in front of the robot
            if self.has_obs:
                self.curr_state = State.OBSTACLE_DETECTED
                # Reset obs handler and set target dist to AVOIDED_DISTANCE in front of current position
                self.obstacle_handler.obstacle_avoided = False
                self.obstacle_handler.update_destination()
                return
            # update last seen direction of person
            self.search_dir = 1 if self.person_loc[0] < 0 else -1
            # Control orientation to face person
            turn_speed = -float(Kp_angle * self.person_loc[0])
            self.cmd_vel_pub.publish(
                Twist(
                    linear=Vector3(x=FORWARD_SPEED, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=turn_speed),
                )
            )
        elif self.curr_state == State.OBSTACLE_DETECTED:
            # Once obstacle is avoided (target pose is reached), switch back to FOLLOWING/NO_PERSON state
            if self.obstacle_handler.obstacle_avoided == True:
                if self.person_loc is not None:
                    self.curr_state = State.FOLLOWING_PERSON
                else:
                    self.curr_state = State.NO_PERSON
                return
            # Use obstacle avoidance handler to navigate past detected object
            self.cmd_vel_pub.publish(self.obstacle_handler.calculate_vel())
            # Place Marker at target goal pose to visualize obstacle avoidance
            pose = Pose(
                position=Point(
                    x=float(self.obstacle_handler.destination[0]),
                    y=float(self.obstacle_handler.destination[1]),
                    z=0.0,
                )
            )
            header = Header(
                stamp=self.get_clock().now().to_msg(), frame_id="odom")
            self.marker_pub.publish(
                Marker(
                    header=header,
                    ns="marker",
                    id=0,
                    type=2,
                    action=0,
                    pose=pose,
                    scale=Vector3(x=0.2, y=0.2, z=0.2),
                    lifetime=Duration(sec=0),
                    color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
                )
            )


def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateMachine()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
