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
        self.world_name = self.declare_parameter(
            'world_name',
            'forest_world'
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
        MODEL_PREFIX = "fire_"
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
        detected_fire_id = None
        closest_fire_pose = (0.0, 0.0, 0.0)

        # Draw bounding boxes for visualization
        vis_image = hsv_image.copy()
        if results and len(results) > 0:
            boxes = results[0].boxes
            # Find the closest unsuppressed fire and only publish that one
            min_dist = float('inf')
            best_id = None
            best_pose = (0.0, 0.0, 0.0)
            best_conf = 0.0
            best_bbox_x = 0.5
            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.class_names[cls_id]
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy
                # Only draw bounding box for unsuppressed fires
                draw_box = True
                if label.lower() == 'fire':
                    for model_id, fire_pos in self.fire_poses.items():
                        for sx, sy in self.suppressed_fires:
                            if math.hypot(fire_pos[0] - sx, fire_pos[1] - sy) < 1.2:
                                draw_box = False
                                break
                        else:
                            dist = math.hypot(fire_pos[0], fire_pos[1])
                            if dist < min_dist:
                                min_dist = dist
                                best_id = model_id
                                best_pose = fire_pos
                                best_conf = conf
                                best_bbox_x = ((x1 + x2) / 2) / hsv_image.shape[1]
                if draw_box:
                    color = (0, 0, 255) if label.lower() == 'fire' else (0, 255, 0)
                    cv2.rectangle(vis_image, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(vis_image, f"{label} {conf:.2f}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            if best_id is not None:
                # Check if closest fire pose is already suppressed
                suppressed = False
                for sx, sy in self.suppressed_fires:
                    if math.hypot(best_pose[0] - sx, best_pose[1] - sy) < 1.2:
                        suppressed = True
                        self.get_logger().info(f"Fire at ({best_pose[0]:.2f}, {best_pose[1]:.2f}) is suppressed (within 1.2m)")
                        break
                if not suppressed:
                    fire_detected = 1.0
                    max_confidence = best_conf
                    detected_fire_id = best_id
                    closest_fire_pose = best_pose
                    bbox_x = best_bbox_x
                else:
                    fire_detected = 0.0
                    max_confidence = 0.0
                    detected_fire_id = None
                    closest_fire_pose = (0.0, 0.0, 0.0)
                    bbox_x = 0.5

        # Show the image with bounding boxes
        cv2.imshow("YOLO Fire Detection", vis_image)
        cv2.waitKey(1)

        # Publish fire_detected, max_confidence, detected_fire_id (as int if possible, else -1)
        fire_id_num = -1
        if detected_fire_id is not None:
            match = re.search(r'fire_(\d+)', detected_fire_id)
            if match:
                fire_id_num = int(match.group(1))

        # Only publish VLA message if a valid, unsuppressed fire is detected
        if results and len(results) > 0 and fire_detected > 0.5:
            msg_out = Float32MultiArray()
            # Publish: [fire_detected, max_confidence, fire_id_num, bbox_x, fire_x, fire_y, fire_z]
            msg_out.data = [fire_detected, max_confidence, float(fire_id_num), bbox_x, closest_fire_pose[0], closest_fire_pose[1], closest_fire_pose[2]]
            self.vla_pub.publish(msg_out)
            self.get_logger().info(f"Fire detected: {fire_detected}, Confidence: {max_confidence:.2f}, Fire ID: {fire_id_num}, bbox_x: {bbox_x:.2f}, pose: {closest_fire_pose}")

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