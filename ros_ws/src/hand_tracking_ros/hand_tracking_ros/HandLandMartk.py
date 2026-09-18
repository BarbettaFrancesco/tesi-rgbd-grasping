##!/usr/bin/env python3
"""
hand_landmark_3d_node.py

Nodo ROS2 unificato che sostituisce la coppia hand_tracker.py +
hand_landmark_3d_node.py.

Pipeline:
  RGB + Depth + CameraInfo (sincronizzati con message_filters)
      -> MediaPipe Tasks API (HandLandmarker)
      -> deproiezione (u, v, depth) -> (x, y, z) metrici
      -> pubblica PoseArray 3D + immagine annotata

Requisiti:
  pip install mediapipe --upgrade
  Scaricare il modello .task:
    wget -O hand_landmarker.task \
      https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
  e passarne il path tramite il parametro ROS 'model_path'.
"""

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision
from ament_index_python.packages import get_package_share_directory
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy

import message_filters
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header


class HandLandmark3DNode(Node):
    def __init__(self):
        super().__init__("hand_landmark_3d_node")

        self.bridge = CvBridge()

        # ---------------- Parametri ROS ----------------
        default_model_path = os.path.join(
            get_package_share_directory('hand_tracking_ros'),
            'hand_landmarker.task'
        )
        self.declare_parameter("model_path", default_model_path)

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("landmarks3d_topic", "/hand_landmarks_3d")
        self.declare_parameter("annotated_topic", "/hand_tracking/image_annotated")
        self.declare_parameter("num_hands", 2)
        self.declare_parameter("min_detection_confidence", 0.6)
        self.declare_parameter("min_tracking_confidence", 0.6)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("depth_scale", 0.001)  # mm -> m (tipico RealSense: 16UC1 in mm)
        self.declare_parameter("publish_annotated", True)
        #self.declare_parameter("use_sim_time", True)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        color_topic = self.get_parameter("color_topic").get_parameter_value().string_value
        depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        caminfo_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        landmarks3d_topic = self.get_parameter("landmarks3d_topic").get_parameter_value().string_value
        annotated_topic = self.get_parameter("annotated_topic").get_parameter_value().string_value
        num_hands = self.get_parameter("num_hands").get_parameter_value().integer_value
        min_det_conf = self.get_parameter("min_detection_confidence").get_parameter_value().double_value
        min_track_conf = self.get_parameter("min_tracking_confidence").get_parameter_value().double_value
        sync_queue_size = self.get_parameter("sync_queue_size").get_parameter_value().integer_value
        sync_slop = self.get_parameter("sync_slop").get_parameter_value().double_value
        self.depth_scale = self.get_parameter("depth_scale").get_parameter_value().double_value
        self.publish_annotated = self.get_parameter("publish_annotated").get_parameter_value().bool_value

        # ---------------- MediaPipe HandLandmarker (Tasks API) ----------------
        base_options = mp_python.BaseOptions(model_asset_path=model_path)
        options = mp_vision.HandLandmarkerOptions(
            base_options=base_options,
            running_mode=mp_vision.RunningMode.VIDEO,
            num_hands=num_hands,
            min_hand_detection_confidence=min_det_conf,
            min_tracking_confidence=min_track_conf,
        )
        self.landmarker = mp_vision.HandLandmarker.create_from_options(options)

        # running_mode=VIDEO richiede timestamp monotonicamente crescenti.
        # Se fai replay non lineare del bag (seek, loop), imposta questo a False
        # per usare detect() invece di detect_for_video() (perdi un po' di
        # ottimizzazione sul tracking tra frame ma eviti errori di timestamp).
        self._last_timestamp_ms = -1

        # ---------------- Subscriber sincronizzati ----------------
        rgb_sub = message_filters.Subscriber(self, Image, color_topic, qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data)
        info_sub = message_filters.Subscriber(self, CameraInfo, caminfo_topic, qos_profile=qos_profile_sensor_data)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub],
            queue_size=sync_queue_size,
            slop=sync_slop,
        )
        self.ts.registerCallback(self.synced_callback)

        # ---------------- Publisher ----------------
        self.pub_3d = self.create_publisher(PoseArray, landmarks3d_topic, 10)

        if self.publish_annotated:
            self.pub_annotated = self.create_publisher(Image, annotated_topic, 10)

        self.get_logger().info("HandLandmark3DNode (unificato, MediaPipe Tasks API) avviato.")
        self.get_logger().info(f"  color_topic:      {color_topic}")
        self.get_logger().info(f"  depth_topic:      {depth_topic}")
        self.get_logger().info(f"  camera_info_topic:{caminfo_topic}")
        self.get_logger().info(f"  landmarks3d_topic:{landmarks3d_topic}")
        self.get_logger().info(f"  annotated_topic:  {annotated_topic}")

        # --- Debug counters ---
        self._dbg_synced_msgs = 0
        self._dbg_hands_detected = 0
        self._dbg_published_msgs = 0

    def synced_callback(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        self._dbg_synced_msgs += 1

        # ---------------- Conversione immagini ----------------
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
        except CvBridgeError as e:
            self.get_logger().error(f"Errore conversione RGB: {e}")
            return

        try:
            # passthrough: mantiene l'encoding originale (tipicamente 16UC1 in mm)
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error(f"Errore conversione depth: {e}")
            return

        height, width = rgb.shape[:2]

        # ---------------- Inferenza MediaPipe ----------------
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        timestamp_ms = int(rgb_msg.header.stamp.sec * 1000 + rgb_msg.header.stamp.nanosec / 1e6)
        # Garantisce timestamp strettamente crescenti (richiesto da detect_for_video)
        if timestamp_ms <= self._last_timestamp_ms:
            timestamp_ms = self._last_timestamp_ms + 1
        self._last_timestamp_ms = timestamp_ms

        result = self.landmarker.detect_for_video(mp_image, timestamp_ms)

        if not result.hand_landmarks:
            self.get_logger().info(
                f"[DEBUG] synced #{self._dbg_synced_msgs}: nessuna mano rilevata "
                f"stamp={rgb_msg.header.stamp.sec}.{rgb_msg.header.stamp.nanosec}"
            )
            if self.publish_annotated:
                self._publish_annotated(rgb, rgb_msg.header)
            return

        self._dbg_hands_detected += 1

        # ---------------- Intrinseci camera ----------------
        K = np.array(info_msg.k).reshape(3, 3)
        fx, fy, cx, cy = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])

        # ---------------- Deproiezione landmark -> 3D metrico ----------------
        poses = PoseArray()
        poses.header = Header()
        poses.header.stamp = rgb_msg.header.stamp
        poses.header.frame_id = info_msg.header.frame_id

        annotated = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if self.publish_annotated else None

        n_valid_points = 0
        n_skipped_points = 0

        for hand_landmarks in result.hand_landmarks:
            for lm in hand_landmarks:
                u = int(lm.x * width)
                v = int(lm.y * height)
                u = int(np.clip(u, 0, width - 1))
                v = int(np.clip(v, 0, height - 1))

                depth_raw = float(depth[v, u])

                if depth_raw <= 0.0 or np.isnan(depth_raw):
                    # Buco nel depth map: nessuna lettura valida per questo pixel.
                    # Qui semplicemente scartiamo il punto. Se in pratica i buchi
                    # sono frequenti, valuta un fallback tipo mediana in una
                    # piccola finestra attorno a (u, v).
                    n_skipped_points += 1
                    if annotated is not None:
                        cv2.circle(annotated, (u, v), 4, (0, 0, 255), -1)  # rosso = depth mancante
                    continue

                z = depth_raw * self.depth_scale  # -> metri
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                pose = Pose()
                pose.position.x = float(x)
                pose.position.y = float(y)
                pose.position.z = float(z)
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                pose.orientation.w = 1.0
                poses.poses.append(pose)
                n_valid_points += 1

                if annotated is not None:
                    cv2.circle(annotated, (u, v), 4, (0, 255, 0), -1)  # verde = punto valido

        self.pub_3d.publish(poses)
        self._dbg_published_msgs += 1

        self.get_logger().info(
            f"[DEBUG] published 3D #{self._dbg_published_msgs}: "
            f"hands={len(result.hand_landmarks)} valid_pts={n_valid_points} "
            f"skipped_pts(no depth)={n_skipped_points} "
            f"stamp={rgb_msg.header.stamp.sec}.{rgb_msg.header.stamp.nanosec}"
        )

        if annotated is not None:
            self._publish_annotated(annotated, rgb_msg.header, already_bgr=True)

    def _publish_annotated(self, image, header, already_bgr=False):
        img_bgr = image if already_bgr else cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Errore conversione immagine annotata: {e}")
            return
        annotated_msg.header = header
        self.pub_annotated.publish(annotated_msg)


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