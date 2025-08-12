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
        
        # Subscription to input image
        self.create_subscription(
            Image, 
            '/test_camera', 
            self.image_callback, 
            10
        )
        
        # Publishers for both processed and HSV images
        self.pub_processed = self.create_publisher(Image, '/processed_image', 10)
        self.pub_hsv = self.create_publisher(Image, '/hsv_image', 10)
        
        self.get_logger().info("🖼️ Image Preprocessor Node initialized")

    # rqt_image_view is no longer launched automatically. Please run it manually if needed.

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            except:
                cv_image_float = self.bridge.imgmsg_to_cv2(msg, '32FC3')
                cv_image = (cv_image_float * 255).astype(np.uint8)
            
            # Process resized image (original functionality)
            processed = cv2.resize(cv_image, (84, 84))
            out_msg_processed = self.bridge.cv2_to_imgmsg(processed, 'rgb8')
            self.pub_processed.publish(out_msg_processed)
            
            # Convert to HSV and publish with correct encoding
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
            out_msg_hsv = self.bridge.cv2_to_imgmsg(hsv_image, 'rgb8')
            self.pub_hsv.publish(out_msg_hsv)
            
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