#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import PointStamped
import math
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
from ultralytics import YOLO
from collections import deque

class YOLOv8FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_node')
        # For smoothing bounding box position and confidence
        self.bbox_x_history = deque(maxlen=10)
        self.confidence_history = deque(maxlen=10)
        self.fire_detection_buffer = deque([False]*2, maxlen=2)
        self.fire_distance_history = deque(maxlen=5)
        self.last_predict_time = 0.0
        self.min_predict_interval = 0.1  # Limit to 10Hz

        # Parameters from YAML
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_path', '/home/adji714/ros2_ws/src/pyroguard/models/best.pt'),
                ('confidence_threshold', 0.7),
            ]
        )
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        # Load YOLOv8 model
        self.model = self.load_model(self.model_path)
        self.class_names = self.model.names
        self.get_logger().info(f"YOLOv8 class names: {self.class_names}")

        # ROS2 Pub/Sub with QoS for dropping old frames
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.image_sub = self.create_subscription(Image, '/hsv_image', self.image_callback, qos_profile=qos)
        self.vla_pub = self.create_publisher(Float32MultiArray, '/vla_output', 10)
        self.fire_distance_sub = self.create_subscription(Float32, '/fire_distance', self.fire_distance_callback, 10)
        self.suppressed_fire_sub = self.create_subscription(Float32MultiArray, '/suppressed_fires', self.suppressed_fire_callback, 10)
        self.fire_world_sub = self.create_subscription(PointStamped, '/fire_position', self.fire_position_callback, 10)
        self.suppressed_world_sub = self.create_subscription(PointStamped, '/suppressed_fire_position', self.suppressed_fire_world_callback, 10)

        self.bridge = CvBridge()
        self.fire_distance = None
        self.suppressed_fires = []
        self.last_detected_fire_world = None
        self.last_detected_fire_time = 0.0
        self.suppressed_world_positions = []
        self.suppressed_ttl = 60.0
        self.get_logger().info("YOLOv8 Fire Detection Node Initialized")

    def fire_distance_callback(self, msg):
        self.fire_distance = msg.data
        self.fire_distance_history.append(self.fire_distance)

    def suppressed_fire_callback(self, msg):
        self.get_logger().debug("Received /suppressed_fires (Float32MultiArray) - ignoring to avoid coord mixing")

    def fire_position_callback(self, msg: PointStamped):
        try:
            self.last_detected_fire_world = (msg.point.x, msg.point.y)
            self.last_detected_fire_time = self.get_clock().now().nanoseconds / 1e9
        except Exception:
            self.last_detected_fire_world = None
            self.last_detected_fire_time = 0.0

    def suppressed_fire_world_callback(self, msg: PointStamped):
        try:
            t = self.get_clock().now().nanoseconds / 1e9
            self.suppressed_world_positions.append((msg.point.x, msg.point.y, t))
            self._prune_suppressed_worlds()
        except Exception:
            pass

    def _prune_suppressed_worlds(self):
        now = self.get_clock().now().nanoseconds / 1e9
        before = len(self.suppressed_world_positions)
        self.suppressed_world_positions = [p for p in self.suppressed_world_positions if now - p[2] <= self.suppressed_ttl]
        after = len(self.suppressed_world_positions)
        if before != after:
            self.get_logger().debug(f"Pruned suppressed_world_positions: {before} -> {after}")

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
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_predict_time < self.min_predict_interval:
            return  # Skip frame to limit to 10Hz
        self.last_predict_time = now
        start_time = self.get_clock().now()

        # Convert ROS Image to OpenCV (HSV with bgr8 encoding)
        hsv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # HSV stored as BGR
        original_height, original_width = hsv_image.shape[:2]

        # Apply slight Gaussian blur to reduce noise
        hsv_image = cv2.GaussianBlur(hsv_image, (3, 3), 0)

        # Process at native resolution
        results = self.model.predict(
            source=hsv_image,
            conf=self.confidence_threshold,
            half=True if torch.cuda.is_available() else False,
            verbose=False
        )

        fire_detected = 0.0
        max_confidence = 0.0
        bbox_x = 0.5
        vis_image = hsv_image.copy()
        all_fires = []

        if results and len(results) > 0:
            boxes = results[0].boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.class_names[cls_id]
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy
                if label.lower() == 'fire':
                    center_x = ((x1 + x2) / 2) / original_width
                    center_y = ((y1 + y2) / 2) / original_height
                    area = (x2 - x1) * (y2 - y1)
                    normalized_area = area / (original_width * original_height)
                    estimated_distance = 1.0 / (normalized_area + 0.001)
                    all_fires.append({
                        'confidence': conf,
                        'center_x': center_x,
                        'center_y': center_y,
                        'bbox': xyxy,
                        'estimated_distance': estimated_distance
                    })
                    color = (0, 255, 255)  # Yellow for all fires
                    cv2.rectangle(vis_image, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(vis_image, f"fire {conf:.2f}", (x1, y1-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            if all_fires:
                all_fires.sort(key=lambda x: x['estimated_distance'])
                nearest_fire = all_fires[0]
                fire_position = (nearest_fire['center_x'], nearest_fire['center_y'])
                is_suppressed = any(self.is_same_fire(fire_position, sup_pos) for sup_pos in self.suppressed_fires)
                if not is_suppressed and self.last_detected_fire_world is not None and self.suppressed_world_positions:
                    for sup_world in self.suppressed_world_positions:
                        dx = abs(self.last_detected_fire_world[0] - sup_world[0])
                        dy = abs(self.last_detected_fire_world[1] - sup_world[1])
                        if math.hypot(dx, dy) < 0.5:
                            is_suppressed = True
                            break
                if not is_suppressed:
                    self.fire_detection_buffer.append(True)
                    self.confidence_history.append(nearest_fire['confidence'])
                    self.bbox_x_history.append(nearest_fire['center_x'])
                    if sum(self.fire_detection_buffer) >= 2:
                        fire_detected = 1.0
                        max_confidence = float(np.mean(self.confidence_history)) if self.confidence_history else nearest_fire['confidence']
                        bbox_x = float(np.mean(self.bbox_x_history)) if self.bbox_x_history else nearest_fire['center_x']
                        x1, y1, x2, y2 = nearest_fire['bbox']
                        color = (0, 0, 255)  # Red for nearest
                        cv2.rectangle(vis_image, (x1, y1), (x2, y2), color, 3)
                        dist_text = f"NEAREST fire {max_confidence:.2f}"
                        cv2.putText(vis_image, dist_text, (x1, y1-30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        self.get_logger().info(f"Nearest fire selected, bbox_x={bbox_x:.2f}, estimated distance={nearest_fire['estimated_distance']:.2f}")
                else:
                    self.fire_detection_buffer.append(False)
                    self.get_logger().info("Nearest fire is already suppressed, ignoring")
            else:
                self.fire_detection_buffer.append(False)
        else:
            self.fire_detection_buffer.append(False)

        green = (0, 255, 0)
        avg_fire_distance = np.mean(self.fire_distance_history) if self.fire_distance_history else 0
        gui_text = f"Confidence: {max_confidence:.2f} | Fire Dist: {avg_fire_distance:.2f} m"
        text_x = 10
        text_y = vis_image.shape[0] - 10
        cv2.putText(vis_image, gui_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, green, 2)

        cv2.imshow("YOLO Fire Detection", vis_image)
        cv2.waitKey(1)

        msg_out = Float32MultiArray()
        msg_out.data = [fire_detected, max_confidence, -1.0, bbox_x]
        self.vla_pub.publish(msg_out)

        inf_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
        if fire_detected > 0.5:
            self.get_logger().info(f"[DEBUG] Inference time: {inf_time:.3f}s | Published VLA output: {msg_out.data}")
        else:
            self.get_logger().warning(f"[DEBUG] Inference time: {inf_time:.3f}s | Published default VLA output (no fire detected): {msg_out.data}")

    def is_same_fire(self, pos1, pos2, threshold=0.05):
        return abs(pos1[0] - pos2[0]) < threshold and abs(pos1[1] - pos2[1]) < threshold

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8FireDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()