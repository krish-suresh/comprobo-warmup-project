import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import math
import numpy as np

Kp_dist = 0.7
Kp_theta = 1.0
SPEED = 0.1
PERSON_DIST = 1.0


class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__("person_follower_node")
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscriber = self.create_subscription(LaserScan, "scan", self.on_scan, 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.dist_to_person = None
        self.angle_to_person = None

    def run_loop(self):
        print(f"angle:{self.angle_to_person}, dist:{self.dist_to_person}")
        if self.dist_to_person is None:
            vels = Twist(
                linear=Vector3(x=SPEED, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=0.0),
            )
        else:
            rotation_speed = self.angle_to_person * Kp_theta
            linear_speed = (self.dist_to_person - PERSON_DIST) * Kp_dist
            vels = Twist(
                linear=Vector3(x=linear_speed, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=rotation_speed),
            )
        self.publisher.publish(vels)

    def on_scan(self, msg: LaserScan):
        angle = 0
        scan_data = []
        for range in msg.ranges:
            if range != 0.0 and not math.isinf(range):
                range_as_point = [math.cos(angle) * range, math.sin(angle) * range]
                scan_data.append(range_as_point)
            angle += msg.angle_increment
        if scan_data:
            scan_centroid = np.array(scan_data).mean(0)
            self.dist_to_person = np.linalg.norm(scan_centroid)
            self.angle_to_person = math.atan2(scan_centroid[1], scan_centroid[0])
            if scan_centroid[0] < 0:
                self.dist_to_person = -self.dist_to_person
                self.angle_to_person = -self.angle_to_person
        else:
            self.dist_to_person = None
            self.angle_to_person = None


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
