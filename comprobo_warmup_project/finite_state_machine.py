import rclpy
from rclpy.node import Node
import numpy as np
from enum import Enum
from geometry_msgs.msg import Twist, Vector3

class State(Enum):
    NO_PERSON = 0
    CLOSE_TO_PERSON = 1
    OBSTACLE_DETECTED = 2
    FOLLOWING_PERSON = 3

class FiniteStateMachine(Node):
    def __init__(self):
        super().__init__('finite_state_machine_node')
        curr_state = State.NO_PERSON
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def update_state(self):
        pass

    def run_loop(self):
        self.update_state()
        if self.curr_state == State.NO_PERSON:
            self.publisher.publish(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.5)))
        if self.curr_state == State.CLOSE_TO_PERSON:
            self.publisher.publish(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)))