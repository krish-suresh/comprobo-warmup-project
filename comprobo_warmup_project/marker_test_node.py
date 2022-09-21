import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Vector3, Point
from builtin_interfaces.msg import Duration


class MarkerTestNode(Node):
    def __init__(self):
        super().__init__("test_marker_node")
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Marker, "test_marker", 10)

    def run_loop(self):
        pose = Pose(position=Point(x=0.0, y=0.0, z=0.0))
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="odom")
        self.publisher.publish(
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
    rclpy.init(args=args)  # Initialize communication with ROS
    node = MarkerTestNode()  # Create our Node
    rclpy.spin(node)  # Run the Node until ready to shutdown
    rclpy.shutdown()  # cleanup


if __name__ == "__main__":
    main()
