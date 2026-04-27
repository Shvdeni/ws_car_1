import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


class CameraViewer(Node):
    def __init__(self):
        super().__init__("camera_viewer")

        self.declare_parameter("image_topic", "/touch_car/front_camera/detections")
        self.declare_parameter("window_name", "Touch car front camera")

        self.image_topic = self.get_parameter("image_topic").value
        self.window_name = self.get_parameter("window_name").value
        self.bridge = CvBridge() if CvBridge is not None else None
        self.window_ready = self._display_available()

        if self.window_ready:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 650, 900)
            self.get_logger().info(f"Showing camera view: {self.image_topic}")
        else:
            self.get_logger().warning(
                "No GUI display found. Connect X/Wayland or run with DISPLAY set "
                "to see the camera window."
            )

        self.create_subscription(Image, self.image_topic, self._on_image, 2)

    def _display_available(self):
        return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))

    def _on_image(self, msg):
        frame = self._image_to_bgr(msg)
        if frame is None or not self.window_ready:
            return

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def _image_to_bgr(self, msg):
        if self.bridge is not None:
            try:
                return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as exc:
                self.get_logger().warning(f"cv_bridge conversion failed: {exc}")

        data = np.frombuffer(msg.data, dtype=np.uint8)
        channels = 4 if msg.encoding.lower() in ("rgba8", "bgra8") else 3
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

    def destroy_node(self):
        if self.window_ready:
            cv2.destroyWindow(self.window_name)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
