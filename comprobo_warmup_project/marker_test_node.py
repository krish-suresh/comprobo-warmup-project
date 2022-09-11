import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Vector3


class MarkerTestNode(Node):
    def __init__(self):
        super().__init__('test_marker_node')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Marker, 'test_marker', 10)

    def run_loop(self):
        pose = Pose(x=1, y=2, z=0)
        self.publisher.publish(Marker(header=Header, ns="marker", id=0,
                               type=2, action=0, pose=pose, scale=Vector3(x=1, y=1, z=1),
                               color=ColorRGBA(r=0.5,g=0.5,b=0)))


def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = MarkerTestNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup


if __name__ == '__main__':
    main()
