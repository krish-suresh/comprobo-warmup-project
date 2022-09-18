import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry
import math
import numpy as np

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

FORCE_PARAMETER = -60
ATTRACTIVE_PARAMETER = 3000
LIN_VEL_SCALE = 10
ANG_VEL_SCALE = 50

class ObstacleAvoider(Node):

    def __init__(self, destination = np.array([0, -2]), use_timer=True):
        super().__init__('obstacle_avoider_node')
        if use_timer:
            timer_period = 0.1
            self.timer = self.create_timer(timer_period, self.run_loop)
        self.subscriber_scan = self.create_subscription(LaserScan, 'scan', self.calculate_repellent_forces, 10)
        self.subscriber_odom = self.create_subscription(Odometry, 'odom', self.update_curr_pose, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.curr_pos = np.array([0, 0])
        self.attractive_forces = np.array([0, 0])
        self.repellent_forces = np.array([0, 0])
        self.heading = np.array([0, 0])
        self.curr_heading = np.array([0, 0])
        self.destination = destination

    def update_destination(self, new_destination):
        self.destination = new_destination

    def calculate_heading(self):
        self.calculate_net_force()
        self.heading = self.net_force/np.linalg.norm(self.net_force)

    def calculate_net_force(self):
        self.net_force = self.attractive_forces + self.repellent_forces

    def calculate_attractive_forces(self):
        self.attractive_forces = ATTRACTIVE_PARAMETER/self.dist_from_target*(self.destination - self.curr_pos)

    def calculate_repellent_forces(self, msg: LaserScan):
        forces = []
        for angle, point in enumerate(msg.ranges):
            if point > 0.0:
                forces.append([FORCE_PARAMETER*math.cos(math.radians(angle))/point**2, FORCE_PARAMETER*math.sin(math.radians(angle))])
        self.repellent_forces = np.sum(np.array(forces), axis=0)

    def update_curr_pose(self, msg: Odometry):
        self.curr_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.orientation = euler_from_quaternion(msg.pose.pose.orientation)
        self.curr_heading = np.array([math.cos(self.orientation), math.sin(self.orientation)])
        self.calculate_attractive_forces()

    @property
    def dist_from_target(self):
        return np.linalg.norm(self.destination - self.curr_pos)

    @property
    def angular_diff(self):
        return math.degrees(np.arcsin(np.cross(self.curr_heading, self.heading)))

    def run_loop(self):
        self.calculate_heading()
        print(self.dist_from_target)
        if self.dist_from_target < 0.01:
            self.publisher.publish(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)))
            rclpy.shutdown()
        self.publisher.publish(Twist(linear=Vector3(x=np.linalg.norm(self.dist_from_target)/LIN_VEL_SCALE, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=self.angular_diff/ANG_VEL_SCALE)))

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()