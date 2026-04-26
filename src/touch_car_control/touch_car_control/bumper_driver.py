import math
import random
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node

try:
    from ros_gz_interfaces.msg import Contacts as ContactMessage
except ImportError:
    from gazebo_msgs.msg import ContactsState as ContactMessage


class DriveState(Enum):
    FORWARD = "forward"
    BACKING = "backing"
    TURNING = "turning"


class BumperDriver(Node):
    def __init__(self):
        super().__init__("bumper_driver")

        self.declare_parameter("forward_speed", 0.08)
        self.declare_parameter("reverse_speed", 0.04)
        self.declare_parameter("turn_speed", 0.9)
        self.declare_parameter("backup_distance", 0.05)
        self.declare_parameter("min_turn_degrees", 35.0)
        self.declare_parameter("max_turn_degrees", 145.0)
        self.declare_parameter("room_limit", 0.60)

        self.forward_speed = self.get_parameter("forward_speed").value
        self.reverse_speed = self.get_parameter("reverse_speed").value
        self.turn_speed = self.get_parameter("turn_speed").value
        self.backup_distance = self.get_parameter("backup_distance").value
        self.min_turn = math.radians(self.get_parameter("min_turn_degrees").value)
        self.max_turn = math.radians(self.get_parameter("max_turn_degrees").value)
        self.room_limit = self.get_parameter("room_limit").value

        self.state = DriveState.FORWARD
        self.state_until = self.get_clock().now()
        self.turn_direction = 1.0

        self.cmd_pub = self.create_publisher(Twist, "/model/touch_car/cmd_vel", 10)
        self.create_subscription(
            ContactMessage,
            "/touch_car/bumper_contacts",
            self._on_bumper,
            10,
        )
        self.create_subscription(
            Odometry,
            "/model/touch_car/odometry",
            self._on_odometry,
            10,
        )
        self.create_timer(0.05, self._on_timer)

    def _on_bumper(self, msg):
        contact_count = len(getattr(msg, "contacts", getattr(msg, "states", [])))
        if self.state != DriveState.FORWARD or contact_count == 0:
            return

        self._start_backup("Bumper contact received.")

    def _on_odometry(self, msg):
        if self.state != DriveState.FORWARD:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if abs(x) >= self.room_limit or abs(y) >= self.room_limit:
            self._start_backup("Bumper fallback reached room boundary.")

    def _start_backup(self, reason):
        now = self.get_clock().now()
        backup_seconds = self.backup_distance / max(self.reverse_speed, 0.001)
        self.state = DriveState.BACKING
        self.state_until = now + Duration(seconds=backup_seconds)
        self.get_logger().info(f"{reason} Backing up 5 cm.")

    def _on_timer(self):
        now = self.get_clock().now()

        if self.state == DriveState.BACKING and now >= self.state_until:
            turn_angle = random.uniform(self.min_turn, self.max_turn)
            self.turn_direction = random.choice((-1.0, 1.0))
            turn_seconds = turn_angle / max(self.turn_speed, 0.001)
            self.state = DriveState.TURNING
            self.state_until = now + Duration(seconds=turn_seconds)
            self.get_logger().info(
                f"Turning {math.degrees(turn_angle):.1f} degrees."
            )
        elif self.state == DriveState.TURNING and now >= self.state_until:
            self.state = DriveState.FORWARD

        self.cmd_pub.publish(self._command_for_state())

    def _command_for_state(self):
        cmd = Twist()
        if self.state == DriveState.FORWARD:
            cmd.linear.x = self.forward_speed
        elif self.state == DriveState.BACKING:
            cmd.linear.x = -self.reverse_speed
        elif self.state == DriveState.TURNING:
            cmd.angular.z = self.turn_direction * self.turn_speed
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = BumperDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
