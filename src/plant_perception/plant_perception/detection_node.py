# sub to camera topic
# convert image to cvbridge
# hand back image data

#!/usr/bin/env python3
"""ROS 2 node for RTMDet-based plant detection."""

import os
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

try:
    from mmdet.apis import inference_detector, init_detector
except ImportError:
    inference_detector = None
    init_detector = None


class DetectorNode(Node):
    """Receives camera images and runs RTMDet inference in a timer callback."""

    def __init__(self) -> None:
        super().__init__('detector_node')

        self.declare_parameter('camera_topic', '/husky1/camera/image')
        self.declare_parameter(
            'config_path',
            os.path.expanduser(
                '~/git/mmdetection/configs/rtmdet/'
                'rtmdet_tiny_8xb32-300e_coco.py'
            ),
        )
        self.declare_parameter(
            'checkpoint_path',
            os.path.expanduser(
                '~/git/mmdetection/checkpoints/'
                'rtmdet_tiny_8xb32-300e_coco_20220902_112414-78e30dcc.pth'
            ),
        )
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('inference_rate_hz', 2.0)

        self.camera_topic = self.get_parameter(
            'camera_topic'
        ).get_parameter_value().string_value
        self.config_path = self.get_parameter(
            'config_path'
        ).get_parameter_value().string_value
        self.checkpoint_path = self.get_parameter(
            'checkpoint_path'
        ).get_parameter_value().string_value
        self.device = self.get_parameter(
            'device'
        ).get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter(
            'confidence_threshold'
        ).get_parameter_value().double_value
        self.inference_rate_hz = self.get_parameter(
            'inference_rate_hz'
        ).get_parameter_value().double_value

        self.bridge = CvBridge()
        self.model = None
        self.latest_image = None
        self.image_lock = threading.Lock()

        self._initialise_model()

        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10,
        )

        rate = max(self.inference_rate_hz, 0.1)
        self.inference_timer = self.create_timer(
            1.0 / rate,
            self.process_latest_image,
        )

        self.get_logger().info(
            f'Subscribed to camera topic: {self.camera_topic}'
        )

    def _initialise_model(self) -> None:
        """Load RTMDet once when the node starts."""
        if init_detector is None:
            self.get_logger().error(
                'MMDetection is unavailable. Verify mmdet/MMCV/MMEngine '
                'are installed in the ROS Python environment.'
            )
            return

        if not os.path.isfile(self.config_path):
            self.get_logger().error(
                f'Model config not found: {self.config_path}'
            )
            return

        if not os.path.isfile(self.checkpoint_path):
            self.get_logger().error(
                f'Model checkpoint not found: {self.checkpoint_path}'
            )
            return

        try:
            self.model = init_detector(
                self.config_path,
                self.checkpoint_path,
                device=self.device,
            )
            self.get_logger().info(
                f'RTMDet model loaded successfully on {self.device}.'
            )
        except Exception as error:
            self.get_logger().error(
                f'Model initialisation failed: {error}'
            )

    def image_callback(self, message: Image) -> None:
        """Convert incoming ROS image and retain only the newest frame."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )
        except CvBridgeError as error:
            self.get_logger().error(f'Image conversion failed: {error}')
            return

        self.get_logger().debug(
            f'Received image: {cv_image.shape[1]}x{cv_image.shape[0]} '
            f'({cv_image.shape[2]} channels)'
        )

        # Copy preserves the image after this callback returns.
        with self.image_lock:
            self.latest_image = cv_image.copy()

    def process_latest_image(self) -> None:
        """Run inference outside the subscription callback."""
        if self.model is None:
            return

        with self.image_lock:
            if self.latest_image is None:
                return

            image = self.latest_image
            self.latest_image = None

        try:
            result = inference_detector(self.model, image)
            instances = result.pred_instances

            boxes = instances.bboxes.cpu().numpy()
            labels = instances.labels.cpu().numpy()
            scores = instances.scores.cpu().numpy()

            class_names = self.model.dataset_meta.get('classes', [])

            for box, label, score in zip(boxes, labels, scores):
                if float(score) < self.confidence_threshold:
                    continue

                class_name = (
                    class_names[label]
                    if label < len(class_names)
                    else f'class_{label}'
                )

                self.get_logger().info(
                    f'Detection: {class_name}, '
                    f'confidence={float(score):.2f}, '
                    f'bbox=[{box[0]:.1f}, {box[1]:.1f}, '
                    f'{box[2]:.1f}, {box[3]:.1f}]'
                )

        except Exception as error:
            self.get_logger().error(f'Inference failed: {error}')


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = DetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()