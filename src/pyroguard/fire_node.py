#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
from ultralytics import YOLO
from collections import deque

class YOLOv8FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_node')
        # For smoothing bounding box position
        self.bbox_x_history = deque(maxlen=5)  # Changed to deque

        # Parameters
        self.model_path = self.declare_parameter(
            'model_path',
            '/home/adji714/ros2_ws/src/pyroguard/models/best.pt'
        ).value
        self.confidence_threshold = self.declare_parameter(
            'confidence_threshold',
            0.5
        ).value

        # Load YOLOv8 model
        self.model = self.load_model(self.model_path)
        self.class_names = self.model.names
        self.get_logger().info(f"YOLOv8 class names: {self.class_names}")

        # ROS2 Pub/Sub
        self.image_sub = self.create_subscription(Image, '/hsv_image', self.image_callback, 10)
        self.vla_pub = self.create_publisher(Float32MultiArray, '/vla_output', 10)

        self.bridge = CvBridge()
        self.get_logger().info("YOLOv8 Fire Detection Node Initialized")

    def load_model(self, model_path):
        try:
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
            model = YOLO(model_path)
            model.to(device)
            self.get_logger().info(f"Loaded YOLOv8 model from {model_path} on {device}")
            return model
        except Exception as e:
            self.get_logger().error(f"Error loading YOLOv8 model: {e}")
            raise

    def image_callback(self, msg):

        hsv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        results = self.model.predict(
            source=hsv_image,
            conf=self.confidence_threshold,
            verbose=False
        )

        fire_detected = 0.0
        max_confidence = 0.0
        bbox_x = 0.5

        vis_image = hsv_image.copy()
        max_area = 0.0
        best_box = None
        best_conf = 0.0
        best_bbox_x = 0.5

        if results and len(results) > 0:
            boxes = results[0].boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.class_names[cls_id]
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy
                if label.lower() == 'fire':
                    area = (x2 - x1) * (y2 - y1)
                    if area > max_area:
                        max_area = area
                        best_box = xyxy
                        best_conf = conf
                        best_bbox_x = ((x1 + x2) / 2) / hsv_image.shape[1]

            # If a fire is detected, use the best box
            if best_box is not None:
                x1, y1, x2, y2 = best_box
                color = (0, 0, 255)
                cv2.rectangle(vis_image, (x1, y1), (x2, y2), color, 2)
                dist_text = f"fire {best_conf:.2f}"
                cv2.putText(vis_image, dist_text, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                fire_detected = 1.0
                max_confidence = best_conf
                # Smooth bbox_x using moving average
                self.bbox_x_history.append(best_bbox_x)
                bbox_x = float(np.mean(self.bbox_x_history)) if self.bbox_x_history else 0.5
                self.get_logger().info(f"Fire detected, bbox_x={bbox_x:.2f}")

        cv2.imshow("YOLO Fire Detection", vis_image)
        cv2.waitKey(1)

        # Publish VLA message
        msg_out = Float32MultiArray()
        msg_out.data = [fire_detected, max_confidence, -1.0, bbox_x]
        self.vla_pub.publish(msg_out)
        if fire_detected > 0.5:
            self.get_logger().info(f"[DEBUG] Published VLA output: {msg_out.data}")
        else:
            self.get_logger().warning(f"[DEBUG] Published default VLA output (no fire detected): {msg_out.data}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8FireDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()