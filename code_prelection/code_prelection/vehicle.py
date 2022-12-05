from random import random
from math import sin, cos

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from std_msgs.msg import String, Float32, Header
from std_srvs.srv import SetBool
from geometry_msgs.msg import TransformStamped

import tf_transformations


class VehicleNode(Node):
    TICK = 1 / 25

    def __init__(self):
        super().__init__("vehicle")
        self.vehicle_name = self.declare_parameter("vehicle_name", "default").value
        self.state = "RUN"
        self.velocity = 0.0
        self.declared_velocity = 0.0
        self.theta = 0.0
        self.angular_velocity = 0.0
        self.declared_angular_velocity = 0.0
        self.x = 0.0
        self.y = float(self.declare_parameter("y", 0.0).value)

        self._state_publisher = self.create_publisher(String, "state", 1)
        self._velocity_publisher = self.create_publisher(Float32, "velocity", 1)
        self._steer_publisher = self.create_publisher(Float32, "steer", 1)
        self._state_srv = self.create_service(SetBool, "set_state", self._set_state)
        self._velocity_sub = self.create_subscription(
            Float32, "declare_velocity", self._set_velocity, 1
        )
        self._steer_sub = self.create_subscription(
            Float32, "declare_steer", self._set_steer, 1
        )
        self.timer = self.create_timer(self.TICK, self._tick_callback)
        self.br = TransformBroadcaster(self)

    def _set_velocity(self, msg: Float32):
        self.declared_velocity = msg.data

    def _set_steer(self, msg: Float32):
        self.declared_angular_velocity = msg.data

    def _tick_callback(self):
        self._update()
        self._publish_state()
        self._publish_velocity()
        self._publish_steer()
        self._publish_localization()

    def _set_state(self, req, res):
        new_state = "RUN" if req.data else "STOP"
        if self.state == new_state:
            res.success = False
            res.message = "new state is already set"
            return res
        self.state = new_state
        res.success = True
        res.message = "OK"
        return res

    def _update(self):
        if self.state == "RUN":
            if abs(self.declared_velocity - self.velocity) < 0.1:
                self.velocity = self.declared_velocity
            elif self.declared_velocity > self.velocity:
                self.velocity += 0.01
            elif self.declared_velocity < self.velocity:
                self.velocity -= 0.3
            if self.declared_angular_velocity > self.angular_velocity:
                self.angular_velocity += 0.05
            elif self.declared_angular_velocity < self.angular_velocity:
                self.angular_velocity -= 0.05
        else:
            if self.velocity > 0:
                self.velocity -= 0.01
            if self.velocity < 0:
                self.velocity += 0.01

        self.theta += self.velocity * self.angular_velocity
        self.x += self.velocity * cos(self.theta)
        self.y += self.velocity * sin(self.theta)

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self._state_publisher.publish(msg)

    def _publish_velocity(self):
        msg = Float32()
        msg.data = self.velocity
        self._velocity_publisher.publish(msg)

    def _publish_steer(self):
        msg = Float32()
        msg.data = self.theta
        self._steer_publisher.publish(msg)

    def _publish_localization(self):
        t = TransformStamped()
        t.header = self._make_header("map")
        t.child_frame_id = f"{self.vehicle_name}/base_link"

        t.transform.translation.x = self.x + random() * 0.01
        t.transform.translation.y = self.y + random() * 0.01
        t.transform.translation.z = random() * 0.01

        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

    def _make_header(self, frame_id: str) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header


def main(args=None):
    rclpy.init()
    node = VehicleNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
