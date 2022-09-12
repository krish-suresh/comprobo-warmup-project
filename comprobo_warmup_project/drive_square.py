import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion
from nav_msgs.msg import Odometry
import math
import numpy as np

def euler_from_quaternion(quat: Quaternion):
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z + np.pi

class DriveSquareNode(Node):
    states = {
        'forward': Twist(linear=Vector3(x=0.1, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
        'turning': Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.1))
    }

    def __init__(self):
        super().__init__('drive_square_node')
        self.position = Point(x=0.0, y=0.0, z=0.0)
        self.orientation = Quaternion()
        self.init_pos = None
        self.init_or = None
        self.curr_state = 'forward'
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(Odometry, 'odom', self.update_current_pose, 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
    
    def update_current_pose(self, msg: Odometry):
        if self.init_pos is None:
            self.init_pos = msg.pose.pose.position
            self.init_or = msg.pose.pose.orientation
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation

    def dist_travelled(self):
        curr_pos = np.array([self.position.x, self.position.y, self.position.z])
        init_pos = np.array([self.init_pos.x, self.init_pos.y, self.init_pos.z])

        return np.linalg.norm(curr_pos-init_pos)

    def angle_travelled(self):
        _, _, curr_yaw = euler_from_quaternion(self.orientation)
        _, _, init_yaw = euler_from_quaternion(self.init_or)
        return min(np.abs(curr_yaw-init_yaw), np.pi*2-np.abs(curr_yaw-init_yaw))

    def run_loop(self):
        if self.curr_state == 'forward':
            if self.dist_travelled() < 1.0:
                self.publisher.publish(self.states[self.curr_state])
            else:
                self.curr_state = 'turning'
                print('change')
                self.init_or = self.orientation
        else:
            print(self.angle_travelled())
            if self.angle_travelled() < np.pi/2:
                self.publisher.publish(self.states[self.curr_state])
            else:
                self.curr_state = 'forward'
                self.init_pos = self.position
            


def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()