#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
from ultralytics import YOLO
import math
import re

class YOLOv8FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_node')

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

        # ROS2 Pub/Sub
        self.image_sub = self.create_subscription(Image, '/hsv_image', self.image_callback, 10)
        self.vla_pub = self.create_publisher(Float32MultiArray, '/vla_output', 10)
        self.suppressed_fire_sub = self.create_subscription(
            PointStamped, '/suppressed_fire_position', self.suppressed_fire_callback, 10)
        self.fire_pose_sub = self.create_subscription(
            TFMessage, f'/world/{self.world_name}/pose/info', self.fire_pose_callback, 10)

        self.bridge = CvBridge()
        self.suppressed_fires = []  # List of (x, y)
        self.fire_poses = {}  # model_name: (x, y, z)

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

    def suppressed_fire_callback(self, msg):
        self.suppressed_fires.append((msg.point.x, msg.point.y))
        self.get_logger().info(f"Added suppressed fire at ({msg.point.x:.2f}, {msg.point.y:.2f})")

    def fire_pose_callback(self, msg):
        self.fire_poses.clear()
        for transform in msg.transforms:
            frame_id = transform.child_frame_id
            if "fire" in frame_id.lower() or "light" in frame_id.lower():
                model_name = self.resolve_model_name(frame_id)
                if model_name:
                    t = transform.transform.translation
                    for sx, sy in self.suppressed_fires:
                        if math.hypot(t.x - sx, t.y - sy) < 0.5:
                            break
                    else:
                        self.fire_poses[model_name] = (t.x, t.y, t.z)

    def resolve_model_name(self, frame_id):
        MODEL_PREFIX = "fire_model_"
        if re.match(fr"^{MODEL_PREFIX}\d+$", frame_id):
            return frame_id
        if '::' in frame_id:
            parts = frame_id.split('::')
            if re.match(fr"^{MODEL_PREFIX}\d+$", parts[0]):
                return parts[0]
        match = re.search(rf'({MODEL_PREFIX}\d+)', frame_id)
        if match:
            return match.group(1)
        return None

    def image_callback(self, msg):
        hsv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        results = self.model.predict(
            source=hsv_image,
            conf=self.confidence_threshold,
            verbose=False
        )

        fire_detected = 0.0
        max_confidence = 0.0

        if results and len(results) > 0:
            boxes = results[0].boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.class_names[cls_id]
                if label.lower() == 'fire':
                    # Check if detected fire matches a suppressed fire
                    for fire_pos in self.fire_poses.values():
                        for sx, sy in self.suppressed_fires:
                            if math.hypot(fire_pos[0] - sx, fire_pos[1] - sy) < 0.5:
                                break
                        else:
                            if conf > max_confidence:
                                fire_detected = 1.0
                                max_confidence = conf

        msg_out = Float32MultiArray()
        msg_out.data = [fire_detected, max_confidence]
        self.vla_pub.publish(msg_out)
        self.get_logger().info(f"Fire detected: {fire_detected}, Confidence: {max_confidence:.2f}")

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