import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry
import math
import numpy as np


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


# Tuned constants used to scale the net forces on the Neato and the sent
# linear and angular velocities.
FORCE_PARAMETER = -40
ATTRACTIVE_PARAMETER = 5000
LIN_VEL_SCALE = 15
ANG_VEL_SCALE = 50


class ObstacleAvoider(Node):
    """
    ROS Node that, when executed, directs a Neato to a given destination (x,y)
    coordinate, while also avoiding any detected obstacles.
    """

    def __init__(self, destination=np.array([2, 0])):
        super().__init__("obstacle_avoider_node")
        # Creates a timer that continuously calls run_loop to control the
        # Neato's motion.
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        # Subscribes to the Neato's LiDAR data.
        self.subscriber_scan = self.create_subscription(
            LaserScan, "scan", self.calculate_repellent_forces, 1000
        )
        # Subscribes to the Neato's odometry data.
        self.subscriber_odom = self.create_subscription(
            Odometry, "odom", self.update_curr_pose, 10
        )
        # Creates a publisher to send linear and angular velocities to the Neato.
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        # Initializes variables to their initial state.
        self.curr_pos = np.array([0, 0])
        self.attractive_forces = np.array([0, 0])
        self.repellent_forces = np.array([0, 0])
        self.heading = np.array([0, 0])
        self.curr_heading = np.array([0, 0])
        self.destination = destination

    def calculate_heading(self):
        """
        Calculates the necessary heading of the Neato to avoid obstacles.
        """
        self.calculate_net_force()
        if np.linalg.norm(self.net_force) == 0:
            self.heading = np.array([0, 0])
        else:
            self.heading = self.net_force / np.linalg.norm(self.net_force)

    def calculate_net_force(self):
        """
        Calculates the net force acting on the Neato.
        """
        self.net_force = self.attractive_forces + self.repellent_forces

    def calculate_attractive_forces(self):
        """
        Calculates the attractive force (force from the destination) being
        applied to the Neato.
        """
        self.attractive_forces = (
            ATTRACTIVE_PARAMETER
            / self.dist_from_target
            * (self.destination - self.curr_pos)
        )

    def calculate_repellent_forces(self, msg: LaserScan):
        """
        Calculates the repellent forces (forces from any detected obstacles)
        being applied to the Neato.
        """
        forces = []
        for angle, point in enumerate(msg.ranges):
            if point > 0.0:
                forces.append(
                    [
                        FORCE_PARAMETER * math.cos(math.radians(angle)) / point**2,
                        FORCE_PARAMETER * math.sin(math.radians(angle)) / point**2,
                    ]
                )
        self.repellent_forces = np.sum(np.array(forces), axis=0)

    def update_curr_pose(self, msg: Odometry):
        """
        Updates the current position, orientation, and heading of the Neato
        based on the received odometry data.
        """
        self.curr_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.orientation = euler_from_quaternion(msg.pose.pose.orientation)
        self.curr_heading = np.array(
            [math.cos(self.orientation), math.sin(self.orientation)]
        )
        # Calls attractive force as the current position has changed, which
        # will change the corresponding force
        self.calculate_attractive_forces()

    @property
    def dist_from_target(self):
        """
        Property to determine the distance the Neato is from its target.
        """
        return np.linalg.norm(self.destination - self.curr_pos)

    @property
    def angular_diff(self):
        """
        Calculates the angular different between the Neato's current heading
        and intended direction of travel.
        """
        return math.degrees(np.arcsin(np.cross(self.curr_heading, self.heading)))

    def run_loop(self):
        """
        Main loop that determines whether the Neato has reached its target and
        sends velocities to the Neato based on the net force acting on it.
        """
        self.calculate_heading()
        print(self.dist_from_target)
        if self.dist_from_target < 0.05:
            self.publisher.publish(
                Twist(
                    linear=Vector3(x=0.0, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=0.0),
                )
            )
            rclpy.shutdown()
        x_vel = np.linalg.norm(self.dist_from_target) / LIN_VEL_SCALE
        self.publisher.publish(
            Twist(
                linear=Vector3(x=0.15, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=self.angular_diff / ANG_VEL_SCALE),
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
