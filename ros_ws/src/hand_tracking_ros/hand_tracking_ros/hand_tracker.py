import threading

import cv2
import mediapipe as mp
import rclpy

from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header


class HandTracker(Node):
    def __init__(self):
        super().__init__('hand_tracker')
        #serve per convertire immagini ros in OpenCv
        self.bridge = CvBridge()
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils

        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            model_complexity=0,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6
        )

        # Default aggiornato per matchare i topic della rosbag (senza dover passare -p manualmente)
        self.declare_parameter(
            'color_topic',
            '/camera/camera/color/image_raw'
        )

        self.declare_parameter(
            'display_output',
            False
        )

        self.declare_parameter(
            'annotated_topic',
            '/hand_tracking/image_annotated'
        )

        self.declare_parameter(
            'landmarks2d_topic',
            '/hand_tracking/landmarks_2d'
        )

        color_topic = self.get_parameter(
            'color_topic'
        ).get_parameter_value().string_value

        annotated_topic = self.get_parameter(
            'annotated_topic'
        ).get_parameter_value().string_value

        landmarks2d_topic = self.get_parameter(
            'landmarks2d_topic'
        ).get_parameter_value().string_value

        self.display_output = self.get_parameter(
            'display_output'
        ).get_parameter_value().bool_value

        #Subscriber
        self.image_subscription = self.create_subscription(
            Image,
            color_topic,
            self.image_callback,
            qos_profile_sensor_data
        )
        #Publisher
        self.annotated_image_publisher = self.create_publisher(
            Image,
            annotated_topic,
            10
        )

        # Publisher 2D landmarks (normalized -> pixel coords are computed by downstream node or here)
        self.landmarks2d_publisher = self.create_publisher(
            PoseArray,
            landmarks2d_topic,
            10
        )

        self.declare_parameter(
            'processing_period_sec',
            0.066
        )

        self.processing_period_sec = self.get_parameter(
            'processing_period_sec'
        ).get_parameter_value().double_value

        self.frame_count = 0
        self.processed_frame_count = 0
        self._frame_lock = threading.Lock()
        self._latest_msg = None
        self._latest_image = None
        self._timer = self.create_timer(
            self.processing_period_sec,
            self.process_latest_frame
        )

        self.get_logger().info('Nodo hand_tracker avviato correttamente.')
        self.get_logger().info(
            f'In ascolto su: {color_topic}'
        )
        self.get_logger().info(
            f'Pubblico l’immagine annotata su: {annotated_topic}'
        )
        self.get_logger().info(
            f'Pubblico i landmark 2D su: {landmarks2d_topic}'
        )


    # Callback leggera: salva solo l'ultimo frame disponibile.
    def image_callback(self, msg):
        self.frame_count += 1

        try:
            input_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            with self._frame_lock:
                self._latest_msg = msg
                self._latest_image = input_image

        except CvBridgeError as error:
            self.get_logger().error(
                f'Errore nella conversione dell’immagine: {error}'
            )

    def process_latest_frame(self):
        with self._frame_lock:
            if self._latest_msg is None or self._latest_image is None:
                return
            msg = self._latest_msg
            input_image = self._latest_image.copy()

        output_image = input_image.copy()

        rgb_image = cv2.cvtColor(
            input_image,
            cv2.COLOR_BGR2RGB
        )

        results = self.hands.process(rgb_image)
        # DEBUG: quantifica se MediaPipe rileva mani
        has_landmarks = results.multi_hand_landmarks is not None
        self.get_logger().info(f"[DEBUG] MediaPipe multi_hand_landmarks={has_landmarks}")

        if results.multi_hand_landmarks:
            height, width = output_image.shape[:2]

            for hand_landmarks in results.multi_hand_landmarks:
                # draw
                self.mp_drawing.draw_landmarks(
                    output_image,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS
                )

                # publish 2D landmarks as PoseArray:
                # - position.x = u (pixel)
                # - position.y = v (pixel)
                # - position.z = visibility/score (optional, currently 1.0)
                poses = PoseArray()
                poses.header = Header()
                poses.header.stamp = msg.header.stamp
                poses.header.frame_id = msg.header.frame_id

                for lm in hand_landmarks.landmark:
                    u = int(lm.x * width)
                    v = int(lm.y * height)

                    p = Pose()
                    p.position.x = float(u)
                    p.position.y = float(v)
                    p.position.z = 1.0
                    p.orientation.x = 0.0
                    p.orientation.y = 0.0
                    p.orientation.z = 0.0
                    p.orientation.w = 1.0
                    poses.poses.append(p)

                self.landmarks2d_publisher.publish(poses)

        annotated_message = self.bridge.cv2_to_imgmsg(
            output_image,
            encoding='bgr8'
        )

        annotated_message.header = msg.header

        self.annotated_image_publisher.publish(
            annotated_message
        )

        self.processed_frame_count += 1

        if self.display_output:
            cv2.imshow('Hand detection', output_image)
            cv2.waitKey(1)

        
        


def main(args=None):
    rclpy.init(args=args)
    node = HandTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
