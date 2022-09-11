import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import math


Kp_wall = 0.7
Kp_theta = 1.0
SPEED = 0.1
WALL_DIST = 0.45
class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)

    def on_scan(self, msg : LaserScan):
        valid_data = [a for a in msg.ranges if a != 0.0]
        if not valid_data or msg.ranges[315] == 0.0 or msg.ranges[235] == 0:
            print("WALL NOT FOUND")
            vels = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0,y=0.0,z=0.0)),
            self.publisher.publish(vels)
        dist_to_wall = min(valid_data)
        theta_wall = (math.pi/4)-math.atan2(msg.ranges[315], msg.ranges[235])
        rotation_speed = (WALL_DIST-dist_to_wall)*Kp_wall + theta_wall*Kp_theta
        print(f"DIST: {dist_to_wall}, THETA: {theta_wall}, SPEED:{rotation_speed}")
        vels = Twist(linear=Vector3(x=SPEED, y=0.0, z=0.0), angular=Vector3(x=0.0,y=0.0,z=rotation_speed))
        self.publisher.publish(vels)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
