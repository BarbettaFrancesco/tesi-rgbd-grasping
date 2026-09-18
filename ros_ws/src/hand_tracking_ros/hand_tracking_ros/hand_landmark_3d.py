import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import message_filters
import numpy as np
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile


class HandLandmark3DNode(Node):
    def __init__(self):
        super().__init__("hand_landmark_3d_node")

        self.bridge = CvBridge()

        # Last received 2D landmarks (pixel coordinates)
        self.landmarks_2d = None

        # Subscriber for 2D landmarks (published by hand_tracker.py)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(
            PoseArray,
            "/hand_tracking/landmarks_2d",
            self.landmarks_2d_callback,
            qos,
        )

        # Subscribers: 2D landmarks + CameraInfo only (NO depth) to avoid sync issues
        self.color_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.caminfo_callback,
            10,
        )

        # Publisher for 3D landmarks
        self.pub_3d_landmarks = self.create_publisher(PoseArray, "/hand_landmarks_3d", 10)

        self.get_logger().info("HandLandmark3DNode started.")

        # --- Debug counters ---
        self._dbg_2d_msgs = 0
        self._dbg_synced_msgs = 0
        self._dbg_published_msgs = 0

    def landmarks_2d_callback(self, msg: PoseArray):
        # Store the latest 2D landmarks as integer pixel coordinates
        self.landmarks_2d = [(int(p.position.x), int(p.position.y)) for p in msg.poses]

        self._dbg_2d_msgs += 1
        self.get_logger().info(
            f"[DEBUG] 2D received #{self._dbg_2d_msgs}: poses={len(msg.poses)} "
            f"sample={(self.landmarks_2d[0] if len(self.landmarks_2d) > 0 else None)}"
        )

    def caminfo_callback(self, cam_info_msg: CameraInfo):
        # Debug: count camera_info sync events
        self._dbg_synced_msgs += 1

        # If we don't have 2D landmarks yet, skip publishing
        if self.landmarks_2d is None or len(self.landmarks_2d) == 0:
            l2d_len = None if self.landmarks_2d is None else len(self.landmarks_2d)
            self.get_logger().info(
                f"[DEBUG] caminfo #{self._dbg_synced_msgs}: skip "
                f"(landmarks_2d_len={l2d_len}) stamp={cam_info_msg.header.stamp.sec}."
                f"{cam_info_msg.header.stamp.nanosec}"
            )
            return

        K = np.array(cam_info_msg.k).reshape(3, 3)
        fx = float(K[0, 0])
        fy = float(K[1, 1])
        cx = float(K[0, 2])
        cy = float(K[1, 2])

        # Without depth we cannot compute real metric 3D.
        # We publish x,y as "ray direction scaled by z=NaN" => set z=NaN.
        poses = PoseArray()
        poses.header = Header()
        poses.header.stamp = cam_info_msg.header.stamp
        poses.header.frame_id = cam_info_msg.header.frame_id

        for (u, v) in self.landmarks_2d:
            # Clamp is not possible without image size; keep pixel coords as-is.
            x = (u - cx) / fx
            y = (v - cy) / fy

            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float("nan")

            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            poses.poses.append(pose)

        self.pub_3d_landmarks.publish(poses)
        self._dbg_published_msgs += 1
        self.get_logger().info(
            f"[DEBUG] published 3D(no-depth) #{self._dbg_published_msgs}: poses={len(poses.poses)} "
            f"stamp={cam_info_msg.header.stamp.sec}.{cam_info_msg.header.stamp.nanosec}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = HandLandmark3DNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
