import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import math

# Constants
KP_WALL = 0.7 # coeff to proportionally control the wall distance correction
KP_THETA = 1.0 # coeff to proportionally control the rotational correction
SPEED = 0.1 # (m/s) constant forward speed of the neato 
WALL_DIST = 0.45 # (m) target distance from the wall
THETA_1 = 315  # (deg) 45 degrees after the right side of the neato
THETA_2 = 235  # (deg)45 degrees before the right side of the neato


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.on_scan, 10)

    def on_scan(self, msg: LaserScan):
        """ On recieving a new scan from the neato lidar, control the wheel speeds to orient and follow a wall to the right side of the neato 
        args: 
            msg : LaserScan - lidar scan data from the /scan topic 
        """
        # filter out non existent lidar data
        valid_data = [a for a in msg.ranges if a != 0.0]
        if not valid_data or msg.ranges[THETA_1] == 0.0 or msg.ranges[THETA_2] == 0.0:
            print("WALL NOT FOUND")
            self.set_cmd_vel(0, 0)
            return
        # find the minimum lidar data point to find the orthogonal distance
        dist_to_wall = min(valid_data)
        # calculate the angle between the neato orientation and the wall plane
        theta_wall = (math.pi/4)-math.atan2(msg.ranges[THETA_1], msg.ranges[THETA_2])
        # rotational speed to achieve the target distance and parallel orientation
        rotation_speed = (WALL_DIST-dist_to_wall)*KP_WALL + theta_wall*KP_THETA
        print(
            f"DIST: {dist_to_wall}, THETA: {theta_wall}, SPEED:{rotation_speed}")
        self.set_cmd_vel(SPEED, rotation_speed)

    def set_cmd_vel(self, linear_vel: float, angular_vel: float):
        """ Sets the velocities of the neato by publishing to the /cmd_vel topic
        args: 
            linear_vel : float - m/s forward speed of the neato, in the x direction of the neato frame
            angular_vel : float - rad/s rotating counter clockwise 
        """
        vel = Twist(linear=Vector3(x=linear_vel, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=angular_vel))
        self.cmd_vel_pub.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
