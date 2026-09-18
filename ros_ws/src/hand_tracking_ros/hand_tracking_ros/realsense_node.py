import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class RealSenseNode(Node):
    def __init__(self):
        super().__init__("realsense_node")

        self.camera_topic = self.declare_parameter(
            "camera_topic", "/usb_cam/image_raw"
        ).get_parameter_value().string_value
        self.depth_topic = self.declare_parameter(
            "depth_topic", "/usb_cam/depth/image_raw"
        ).get_parameter_value().string_value
        self.infrared_topic = self.declare_parameter(
            "infrared_topic", "/usb_cam/infrared/image_raw"
        ).get_parameter_value().string_value
        self.publish_color = self.declare_parameter(
            "publish_color", True
        ).get_parameter_value().bool_value
        self.publish_depth = self.declare_parameter(
            "publish_depth", False
        ).get_parameter_value().bool_value
        self.publish_infrared = self.declare_parameter(
            "publish_infrared", False
        ).get_parameter_value().bool_value
        self.color_width = self.declare_parameter(
            "color_width", 640
        ).get_parameter_value().integer_value
        self.color_height = self.declare_parameter(
            "color_height", 480
        ).get_parameter_value().integer_value
        self.depth_width = self.declare_parameter(
            "depth_width", 640
        ).get_parameter_value().integer_value
        self.depth_height = self.declare_parameter(
            "depth_height", 480
        ).get_parameter_value().integer_value
        self.fps = self.declare_parameter(
            "fps", 30
        ).get_parameter_value().integer_value
        self.infrared_index = self.declare_parameter(
            "infrared_index", 1
        ).get_parameter_value().integer_value

        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError(
                "pyrealsense2 is required for realsense_node. "
                "Install the Intel RealSense Python bindings in this environment."
            ) from exc

        self.rs = rs
        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        if self.publish_color:
            self.config.enable_stream(
                rs.stream.color,
                self.color_width,
                self.color_height,
                rs.format.bgr8,
                self.fps,
            )

        if self.publish_depth:
            self.config.enable_stream(
                rs.stream.depth,
                self.depth_width,
                self.depth_height,
                rs.format.z16,
                self.fps,
            )

        if self.publish_infrared:
            self.config.enable_stream(
                rs.stream.infrared,
                self.infrared_index,
                self.depth_width,
                self.depth_height,
                rs.format.y8,
                self.fps,
            )

        self.color_publisher = (
            self.create_publisher(Image, self.camera_topic, 1)
            if self.publish_color
            else None
        )
        self.depth_publisher = (
            self.create_publisher(Image, self.depth_topic, 1)
            if self.publish_depth
            else None
        )
        self.infrared_publisher = (
            self.create_publisher(Image, self.infrared_topic, 1)
            if self.publish_infrared
            else None
        )

        self.pipeline.start(self.config)
        self.timer = self.create_timer(1.0 / float(self.fps), self.timer_callback)

        self.get_logger().info(
            f"Publishing RealSense color={self.publish_color} "
            f"on {self.camera_topic}; depth={self.publish_depth}; "
            f"infrared={self.publish_infrared}"
        )

    def timer_callback(self):
        # poll_for_frames() è non bloccante e aiuta a non rallentare il timer ROS
        frames = self.pipeline.poll_for_frames()
        if not frames:
            return

        stamp = self.get_clock().now().to_msg()

        if self.color_publisher is not None:
            color_frame = frames.get_color_frame()
            if color_frame:
                color_image = np.asanyarray(color_frame.get_data())
                color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                color_msg.header.stamp = stamp
                self.color_publisher.publish(color_msg)
            else:
                self.get_logger().warn("RealSense color frame is not available")

        if self.depth_publisher is not None:
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
                depth_msg.header.stamp = stamp
                self.depth_publisher.publish(depth_msg)

        if self.infrared_publisher is not None:
            infrared_frame = frames.get_infrared_frame(self.infrared_index)
            if infrared_frame:
                infrared_image = np.asanyarray(infrared_frame.get_data())
                infrared_msg = self.bridge.cv2_to_imgmsg(
                    infrared_image, encoding="mono8"
                )
                infrared_msg.header.stamp = stamp
                self.infrared_publisher.publish(infrared_msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RealSenseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
