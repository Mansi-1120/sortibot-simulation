import os
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import numpy as np

# Use TensorFlow Lite only (no tflite-runtime)
from tensorflow import lite as tflite


class SortibotPerceptionNode(Node):
    def __init__(self):
        super().__init__('sortibot_perception')

        self.bridge = CvBridge()

        # -----------------------------------------------------
        # Use model / labels directly from your workspace src
        # -----------------------------------------------------
        models_dir = os.path.expanduser(
            '~/sortibot_ws/src/sortibot_perception/models'
        )
        model_path = os.path.join(models_dir, 'model.tflite')
        labels_path = os.path.join(models_dir, 'labels.txt')

        self.get_logger().info(f'Model path: {model_path}')
        self.get_logger().info(f'Labels path: {labels_path}')

        if not os.path.exists(model_path):
            self.get_logger().error(f'Model file not found: {model_path}')
            return
        if not os.path.exists(labels_path):
            self.get_logger().error(f'Labels file not found: {labels_path}')
            return

        # Log model file size (to be sure it's real, not empty)
        try:
            size_bytes = os.path.getsize(model_path)
            self.get_logger().info(f'Model size: {size_bytes} bytes')
        except Exception as e:
            self.get_logger().warn(f'Could not read model file size: {e}')

        # -----------------------------------------------------
        # Load labels
        # -----------------------------------------------------
        self.labels = []
        try:
            with open(labels_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line:
                        # Edge Impulse labels often like "0 metal"
                        parts = line.split(' ', 1)
                        if len(parts) == 2:
                            self.labels.append(parts[1])
                        else:
                            self.labels.append(parts[0])
        except Exception as e:
            self.get_logger().error(f'Error reading labels: {e}')

        # -----------------------------------------------------
        # Load TFLite model (TensorFlow Lite)
        # -----------------------------------------------------
        try:
            self.interpreter = tflite.Interpreter(model_path=model_path)
            self.interpreter.allocate_tensors()
        except Exception as e:
            self.get_logger().error(f'Failed to load TFLite model: {e}')
            return

        # Get input/output details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # Assume single input
        self.input_height = self.input_details[0]['shape'][1]
        self.input_width = self.input_details[0]['shape'][2]

        self.get_logger().info(
            f'Model loaded. Input size: {self.input_width}x{self.input_height}'
        )

        # -----------------------------------------------------
        # Subscribe to camera
        # -----------------------------------------------------
        self.image_sub = self.create_subscription(
            Image,
            '/sortibot/sortibot_camera/image_raw',
            self.image_callback,
            10
        )

        # Publish detected class (simple)
        self.class_pub = self.create_publisher(
            String,
            '/sortibot/object_class',
            10
        )

        # NEW: high-level control command for motion node
        self.control_pub = self.create_publisher(
            String,
            '/sortibot/control',
            10
        )

    def image_callback(self, msg: Image):
        # Convert ROS image → OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Resize to model input size
        resized = cv2.resize(cv_image, (self.input_width, self.input_height))

        # Edge Impulse FOMO often uses uint8 [0..255], but sometimes float32 [0..1].
        # We will check the input type honestly.
        input_type = self.input_details[0]['dtype']

        if input_type == np.float32:
            input_data = np.asarray(resized, dtype=np.float32) / 255.0
        else:
            input_data = np.asarray(resized, dtype=np.uint8)

        # Add batch dimension
        input_data = np.expand_dims(input_data, axis=0)

        # Set tensor and run inference
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

        # Get output
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        # For FOMO, output_data shape is (grid_cells, num_classes)
        # We will do a simple "dominant class" decision

        # Flatten predictions: for each cell, find best class and score
        # Last class is often "background". We skip it.
        try:
            num_classes = output_data.shape[-1]
            foreground_scores = output_data[..., :num_classes - 1]  # ignore background
            # Find best class per cell
            cell_class = np.argmax(foreground_scores, axis=-1)
            cell_score = np.max(foreground_scores, axis=-1)

            # Threshold
            mask = cell_score > 0.5  # confidence threshold
            if not np.any(mask):
                # Nothing confident enough
                detected_label = 'unknown'
            else:
                # Count most frequent class among confident cells
                selected_classes = cell_class[mask]
                unique, counts = np.unique(selected_classes, return_counts=True)
                best_class_index = unique[np.argmax(counts)]
                if best_class_index < len(self.labels):
                    detected_label = self.labels[best_class_index]
                else:
                    detected_label = f'class_{best_class_index}'
        except Exception as e:
            self.get_logger().warn(f'Error parsing FOMO output: {e}')
            detected_label = 'unknown'

        # Publish result (class label)
        msg_out = String()
        msg_out.data = detected_label
        self.class_pub.publish(msg_out)

        # Optional: log occasionally
        self.get_logger().info(f'Detected: {detected_label}')

        # NEW: publish high-level control command based on detected_label
        ctrl_msg = String()

        if detected_label == 'metal':
            ctrl_msg.data = 'pick_metal'
        elif detected_label == 'plastic':
            ctrl_msg.data = 'pick_plastic'
        elif detected_label == 'paper_wood':
            ctrl_msg.data = 'pick_paper_wood'
        else:
            ctrl_msg.data = 'idle'  # nothing confident / background

        self.control_pub.publish(ctrl_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SortibotPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
