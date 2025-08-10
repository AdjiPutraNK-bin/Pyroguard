#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImagePreprocessorNode(Node):
    def __init__(self):
        super().__init__('image_preprocessor_node')
        self.bridge = CvBridge()
        
        self.create_subscription(
            Image, 
            '/test_camera', 
            self.image_callback, 
            10
        )
        
        self.pub = self.create_publisher(Image, '/processed_image', 10)
        
        self.get_logger().info("🖼️ Image Preprocessor Node initialized")

        # Launch rqt_image_view for /processed_image
        import subprocess
        try:
            subprocess.Popen(["rqt_image_view", "-t", "/processed_image"])
            self.get_logger().info("Launched rqt_image_view for /processed_image")
        except Exception as e:
            self.get_logger().warn(f"Could not launch rqt_image_view: {e}")

    def image_callback(self, msg):
        try:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            except:
                cv_image_float = self.bridge.imgmsg_to_cv2(msg, '32FC3')
                cv_image = (cv_image_float * 255).astype(np.uint8)
            
            processed = cv2.resize(cv_image, (84, 84))
            out_msg = self.bridge.cv2_to_imgmsg(processed, 'rgb8')
            self.pub.publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f'Image processing failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePreprocessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()