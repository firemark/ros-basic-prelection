import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformException, TransformListener
from .tf2_geometry_msgs import do_transform_point

from std_msgs.msg import Float32
from nav_msgs.msg import GridCells
from geometry_msgs.msg import TransformStamped, PointStamped


class AutonomyControllerNode(Node):
    def __init__(self):
        super().__init__("autonomy_controller")
        self.vehicle_name = self.declare_parameter("vehicle_name", "bb1").value
        self._velocity_publisher = self.create_publisher(
            Float32, f"vehicle/{self.vehicle_name}/declare_velocity", 1
        )
        self._obstacle_subscription = self.create_subscription(
            GridCells, "obstacles", self.callback, 1
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        print("START")

    def callback(self, msg: GridCells):
        transform = self.find_transformation()
        if transform is None:
            return
        is_any_danger_obstacle = self.check_obstacles(msg, transform)
        self.set_velocity(0.0 if is_any_danger_obstacle else 1.0)

    def find_transformation(self) -> TransformStamped:
        try:
            now = rclpy.time.Time()
            return self.tf_buffer.lookup_transform(
                f"{self.vehicle_name}/base_link", "map", now
            )
        except TransformException as ex:
            self.get_logger().warning(f"Could not transform: {ex}")
            return None

    def set_velocity(self, velocity: float):
        msg = Float32()
        msg.data = velocity
        self._velocity_publisher.publish(msg)

    def check_obstacles(
        self,
        msg: GridCells,
        transform: TransformStamped,
        tunnel_width=5.0,
        safe_distance=30.0,
    ) -> bool:
        def do_transform(p):
            stamped = PointStamped()
            stamped.header = transform.header
            stamped.point = p
            return do_transform_point(stamped, transform).point

        points = map(do_transform, msg.cells)
        tunnel_points = (p for p in points if abs(p.y) < tunnel_width)
        return any(0.0 < p.x < safe_distance for p in tunnel_points)


def main(args=None):
    rclpy.init()
    node = AutonomyControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
