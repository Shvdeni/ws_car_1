import math
import random
from enum import Enum
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

try:
    from ros_gz_interfaces.msg import Contacts as ContactMessage
except ImportError:
    from gazebo_msgs.msg import ContactsState as ContactMessage

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class DriveState(Enum):
    SCANNING = "scanning"
    APPROACHING = "approaching"
    BACKING = "backing"
    TURNING = "turning"


class BumperDriver(Node):
    def __init__(self):
        super().__init__("bumper_driver")

        default_model_path = (
            Path(get_package_share_directory("touch_car_control"))
            / "models"
            / "best.pt"
        )

        self.declare_parameter("model_path", str(default_model_path))
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("forward_speed", 0.08)
        self.declare_parameter("approach_speed", 0.07)
        self.declare_parameter("reverse_speed", 0.04)
        self.declare_parameter("scan_speed", 0.45)
        self.declare_parameter("turn_speed", 0.9)
        self.declare_parameter("backup_distance", 0.05)
        self.declare_parameter("min_turn_degrees", 35.0)
        self.declare_parameter("max_turn_degrees", 145.0)
        self.declare_parameter("room_limit", 0.60)
        self.declare_parameter("sphere_limit", 0.55)
        self.declare_parameter("collect_distance", 0.20)
        self.declare_parameter("spawn_clearance", 0.35)
        self.declare_parameter("collect_box_height_fraction", 0.35)
        self.declare_parameter("respawn_delay", 0.8)
        self.declare_parameter("image_width", 650)
        self.declare_parameter("image_height", 900)

        self.model_path = Path(self.get_parameter("model_path").value)
        self.confidence_threshold = self.get_parameter("confidence_threshold").value
        self.forward_speed = self.get_parameter("forward_speed").value
        self.approach_speed = self.get_parameter("approach_speed").value
        self.reverse_speed = self.get_parameter("reverse_speed").value
        self.scan_speed = self.get_parameter("scan_speed").value
        self.turn_speed = self.get_parameter("turn_speed").value
        self.backup_distance = self.get_parameter("backup_distance").value
        self.min_turn = math.radians(self.get_parameter("min_turn_degrees").value)
        self.max_turn = math.radians(self.get_parameter("max_turn_degrees").value)
        self.room_limit = self.get_parameter("room_limit").value
        self.sphere_limit = self.get_parameter("sphere_limit").value
        self.collect_distance = self.get_parameter("collect_distance").value
        self.spawn_clearance = self.get_parameter("spawn_clearance").value
        self.collect_box_height_fraction = self.get_parameter(
            "collect_box_height_fraction"
        ).value
        self.respawn_delay = self.get_parameter("respawn_delay").value
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)

        self.state = DriveState.SCANNING
        self.state_until = self.get_clock().now()
        self.turn_direction = 1.0
        self.last_detection = None
        self.last_detection_time = None
        self.last_image_time = None
        self.latest_pose = None
        self.sphere_pose = None
        self.sphere_visible = False
        self.sphere_pose_pending = False
        self.respawn_at = self.get_clock().now()
        self.score = 0

        self.bridge = CvBridge() if CvBridge is not None else None
        self.model = self._load_model()

        self.cmd_pub = self.create_publisher(Twist, "/model/touch_car/cmd_vel", 10)
        self.annotated_image_pub = self.create_publisher(
            Image,
            "/touch_car/front_camera/detections",
            2,
        )
        self.set_pose_client = self.create_client(
            SetEntityPose,
            "/world/room_1_5m/set_pose",
        )

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
        self.create_subscription(
            Image,
            "/touch_car/front_camera/image",
            self._on_image,
            2,
        )
        self.create_timer(0.05, self._on_timer)
        self.create_timer(1.0, self._check_camera)
        self.create_timer(0.5, self._ensure_sphere_pose)

        self.get_logger().info("Scanning for the red sphere.")

    def _load_model(self):
        if YOLO is None:
            self.get_logger().warning(
                "Ultralytics is not installed; using HSV red detector fallback."
            )
            return None
        if not self.model_path.exists():
            self.get_logger().warning(
                f"Model file not found at {self.model_path}; using HSV fallback."
            )
            return None

        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        return YOLO(str(self.model_path))

    def _on_image(self, msg):
        self.last_image_time = self.get_clock().now()
        if not self.sphere_visible:
            return

        frame = self._image_to_bgr(msg)
        if frame is None:
            return

        detection = self._detect_with_model(frame)
        if detection is None:
            detection = self._detect_red_blob(frame)

        self._publish_annotated_image(msg, frame, detection)

        if detection is None:
            return

        self.last_detection = detection
        self.last_detection_time = self.get_clock().now()
        if self._detection_is_close(detection):
            self._collect_sphere("Camera reached red sphere.")
            return

        if self.state == DriveState.SCANNING:
            self.state = DriveState.APPROACHING
            self.get_logger().info("Red sphere detected. Approaching.")

    def _publish_annotated_image(self, source_msg, frame, detection):
        annotated = frame.copy()
        if detection is not None:
            x1 = int(detection["x1"])
            y1 = int(detection["y1"])
            x2 = int(detection["x2"])
            y2 = int(detection["y2"])
            cx = int(detection["cx"])
            cy = int(detection["cy"])
            label = (
                f"class {detection['class']} "
                f"conf {detection['confidence']:.2f}"
            )

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cv2.circle(annotated, (cx, cy), 7, (255, 0, 0), -1)
            cv2.putText(
                annotated,
                label,
                (x1, max(28, y1 - 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            annotated,
            f"score {self.score} | state {self.state.value}",
            (18, 38),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        if self.bridge is not None:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        else:
            out_msg = Image()
            out_msg.height = annotated.shape[0]
            out_msg.width = annotated.shape[1]
            out_msg.encoding = "bgr8"
            out_msg.is_bigendian = False
            out_msg.step = annotated.shape[1] * 3
            out_msg.data = annotated.tobytes()

        out_msg.header = source_msg.header
        self.annotated_image_pub.publish(out_msg)

    def _image_to_bgr(self, msg):
        if self.bridge is not None:
            try:
                return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as exc:
                self.get_logger().warning(f"cv_bridge conversion failed: {exc}")

        data = np.frombuffer(msg.data, dtype=np.uint8)
        channels = 3
        if msg.encoding.lower() in ("rgba8", "bgra8"):
            channels = 4
        expected = msg.height * msg.width * channels
        if data.size < expected:
            return None

        frame = data[:expected].reshape((msg.height, msg.width, channels))
        encoding = msg.encoding.lower()
        if encoding == "rgb8":
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if encoding == "rgba8":
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame[:, :, :3]

    def _detect_with_model(self, frame):
        if self.model is None:
            return None

        results = self.model.predict(
            source=frame,
            conf=self.confidence_threshold,
            verbose=False,
        )
        best = None
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                confidence = float(box.conf[0])
                cls = int(box.cls[0])
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                item = {
                    "class": cls,
                    "confidence": confidence,
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "cx": cx,
                    "cy": cy,
                }
                if best is None or confidence > best["confidence"]:
                    best = item
        return best

    def _detect_red_blob(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red_1 = np.array([0, 90, 80])
        upper_red_1 = np.array([12, 255, 255])
        lower_red_2 = np.array([168, 90, 80])
        upper_red_2 = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask |= cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < 80:
            return None

        x, y, w, h = cv2.boundingRect(contour)
        confidence = min(0.99, area / float(frame.shape[0] * frame.shape[1]) * 25.0)
        return {
            "class": 0,
            "confidence": confidence,
            "x1": float(x),
            "y1": float(y),
            "x2": float(x + w),
            "y2": float(y + h),
            "cx": float(x + w / 2.0),
            "cy": float(y + h / 2.0),
        }

    def _detection_is_close(self, detection):
        box_height = detection["y2"] - detection["y1"]
        return box_height >= self.image_height * self.collect_box_height_fraction

    def _on_bumper(self, msg):
        contacts = getattr(msg, "contacts", getattr(msg, "states", []))
        contact_count = len(contacts)
        if contact_count == 0 or self.state not in (
            DriveState.SCANNING,
            DriveState.APPROACHING,
        ):
            return

        if self._contact_mentions_sphere(contacts) or self._recently_chasing_sphere():
            self._collect_sphere("Bumper touched red sphere.")
            return

        self._start_backup("Bumper contact received.")

    def _contact_mentions_sphere(self, contacts):
        for contact in contacts:
            if "red_sphere" in str(contact):
                return True
        return False

    def _recently_chasing_sphere(self):
        if self.state != DriveState.APPROACHING or self.last_detection_time is None:
            return False

        return self.get_clock().now() - self.last_detection_time <= Duration(seconds=1.5)

    def _on_odometry(self, msg):
        self.latest_pose = msg.pose.pose

        if self._is_close_to_sphere():
            self._collect_sphere("Reached red sphere.")
            return

        if self.state not in (DriveState.SCANNING, DriveState.APPROACHING):
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if abs(x) >= self.room_limit or abs(y) >= self.room_limit:
            self._start_backup("Bumper fallback reached room boundary.")

    def _is_close_to_sphere(self):
        if (
            not self.sphere_visible
            or self.latest_pose is None
            or self.sphere_pose is None
        ):
            return False

        dx = self.latest_pose.position.x - self.sphere_pose.position.x
        dy = self.latest_pose.position.y - self.sphere_pose.position.y
        return math.hypot(dx, dy) <= self.collect_distance

    def _start_backup(self, reason):
        now = self.get_clock().now()
        backup_seconds = self.backup_distance / max(self.reverse_speed, 0.001)
        self.state = DriveState.BACKING
        self.state_until = now + Duration(seconds=backup_seconds)
        self.get_logger().info(f"{reason} Backing up 5 cm.")

    def _collect_sphere(self, reason):
        if self.state == DriveState.BACKING:
            return

        self.score += 1
        self.last_detection = None
        self.last_detection_time = None
        self.state = DriveState.SCANNING
        self.sphere_visible = False
        self.sphere_pose_pending = False
        self.respawn_at = self.get_clock().now() + Duration(seconds=self.respawn_delay)
        self.get_logger().info(
            f"{reason} Score: {self.score}. Red sphere disappeared."
        )
        self._hide_sphere()

    def _ensure_sphere_pose(self):
        if self.sphere_visible or self.sphere_pose_pending:
            return

        if self.get_clock().now() >= self.respawn_at:
            self._place_sphere_randomly()

    def _hide_sphere(self):
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = -1.0
        pose.orientation.w = 1.0
        self._set_sphere_pose(pose, visible=False)

    def _place_sphere_randomly(self):
        pose = Pose()
        for _ in range(40):
            pose.position.x = random.uniform(-self.sphere_limit, self.sphere_limit)
            pose.position.y = random.uniform(-self.sphere_limit, self.sphere_limit)
            if self._spawn_pose_is_clear(pose):
                break
        pose.position.z = 0.045
        pose.orientation.w = 1.0
        self._set_sphere_pose(pose, visible=True)

    def _spawn_pose_is_clear(self, pose):
        if math.hypot(pose.position.x, pose.position.y) <= 0.22:
            return False

        if self.latest_pose is None:
            return True

        dx = pose.position.x - self.latest_pose.position.x
        dy = pose.position.y - self.latest_pose.position.y
        return math.hypot(dx, dy) >= self.spawn_clearance

    def _set_sphere_pose(self, pose, visible):
        if not self.set_pose_client.service_is_ready():
            self.set_pose_client.wait_for_service(timeout_sec=0.0)
            return

        self.sphere_pose_pending = True
        request = SetEntityPose.Request()
        request.entity = Entity(name="red_sphere", type=Entity.MODEL)
        request.pose = pose
        future = self.set_pose_client.call_async(request)
        future.add_done_callback(
            lambda result: self._on_sphere_pose_set(result, pose, visible)
        )

    def _on_sphere_pose_set(self, future, pose, visible):
        try:
            response = future.result()
        except Exception as exc:
            self.sphere_pose_pending = False
            self.get_logger().warning(f"Failed to move red sphere: {exc}")
            return

        self.sphere_pose_pending = False
        if response.success:
            self.sphere_pose = pose
            self.sphere_visible = visible
            if visible:
                self.last_detection = None
                self.last_detection_time = None
                self.state = DriveState.SCANNING
                self.get_logger().info(
                    f"Red sphere appeared at x={pose.position.x:.2f}, "
                    f"y={pose.position.y:.2f}. Scanning."
                )
        else:
            self.get_logger().warning("Gazebo rejected red sphere pose update.")

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
            self.state = DriveState.SCANNING

        self.cmd_pub.publish(self._command_for_state(now))

    def _command_for_state(self, now):
        cmd = Twist()
        if self.state == DriveState.SCANNING:
            cmd.angular.z = self.scan_speed
        elif self.state == DriveState.APPROACHING:
            self._fill_approach_command(cmd, now)
        elif self.state == DriveState.BACKING:
            cmd.linear.x = -self.reverse_speed
        elif self.state == DriveState.TURNING:
            cmd.angular.z = self.turn_direction * self.turn_speed
        return cmd

    def _fill_approach_command(self, cmd, now):
        if self.last_detection is None or self.last_detection_time is None:
            self.state = DriveState.SCANNING
            return

        if now - self.last_detection_time > Duration(seconds=0.8):
            self.state = DriveState.SCANNING
            self.last_detection = None
            return

        error = (self.last_detection["cx"] - (self.image_width / 2.0)) / (
            self.image_width / 2.0
        )
        error = max(-1.0, min(1.0, error))
        cmd.angular.z = -error * 0.8
        cmd.linear.x = self.approach_speed * max(0.25, 1.0 - abs(error))

    def _check_camera(self):
        if self.last_image_time is None:
            self.get_logger().warning(
                "Waiting for /touch_car/front_camera/image from Gazebo."
            )


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
