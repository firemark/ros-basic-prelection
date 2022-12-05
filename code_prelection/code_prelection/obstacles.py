from random import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point


class ObstaclesNode(Node):
    TICK = 1 / 25

    def __init__(self):
        super().__init__("obstacles")
        self._obstacles_publisher = self.create_publisher(GridCells, "obstacles", 1)
        self.timer = self.create_timer(self.TICK, self._tick_callback)
        self.declare_parameter("count_obstacles", 100)

    def _tick_callback(self):
        msg = GridCells()
        msg.header = self._make_header("map")
        msg.cell_width = 15.0
        msg.cell_height = 1.0
        count_obstacles = self.get_parameter("count_obstacles").value

        for y in [-20, 80]:
            for x in range(0, count_obstacles):
                point_msg = Point()
                point_msg.x = x * 20 + random() * 0.05
                point_msg.y = y + random() * 0.05
                point_msg.z = 0.0
                msg.cells.append(point_msg)

        for y in range(-3, 20, 6):
            obstacle_point_msg = Point()
            obstacle_point_msg.x = 350 + random() * 0.5
            obstacle_point_msg.y = y + random() * 0.5
            obstacle_point_msg.z = random() * 0.5
            msg.cells.append(obstacle_point_msg)

        self._obstacles_publisher.publish(msg)

    def _make_header(self, frame_id: str) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header


def main(args=None):
    rclpy.init()
    node = ObstaclesNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
