import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from ultralytics import YOLO

from ament_index_python.packages import get_package_share_directory
import os

import time

class YOLODetector(Node):
    
    def __init__(self):
        super().__init__('yolo_detector')
        self.publisher_ = self.create_publisher(Image, '/image_output_topic', 10)
        self.subscriber = self.create_subscription(Image, '/image_input_topic', self.image_callback, 10) #calls image_callback when receive message

        # Declare a ROS parameter
        self.declare_parameter('rate_limit', 1.0)
        # Get Parameter value
        rate_limit = self.get_parameter('rate_limit').get_parameter_value().double_value
        self.get_logger().info("Detection Rate Limit: %.2f"%rate_limit)
        self.last_detection_time = time.time()
        self.detection_interval = 1.0/rate_limit # minimum time between detections

        # Set package installation directory
        pkg_dir = get_package_share_directory("yolo_detector")
        # Full path to model file (Within installation directory)
        model_path = os.path.join(pkg_dir, 'models', 'yolov8n.pt')
        # Initialize the YOLOv8 model
        self.get_logger().info("Loading YOLO model: " + model_path)
        self.model = YOLO(model_path)

        #Initialise YOLOv8 model
        self.model = YOLO('yolov8n.pt')

        #Bridge to convert ROS to OpenCV
        self.bridge = CvBridge()

        self.get_logger().info("YOLO Detector Node Started!")

    def image_callback(self, msg):
        # Control detection frame rate
        if (self.detection_interval > (time.time() - self.last_detection_time)):
            # If a detection_interval has not yet passed since last detection, ignore image
            return
        self.last_detection_time = time.time()
        self.get_logger().info("Received an Image")
        # TODO : Implement YOLO Detection Code

        # Convert the incoming ROS image to an RGB NumPy array
        frame_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        # Run YOLO inference on this RGB image
        results = self.model(frame_rgb, verbose=False)
        # results[0].plot(show=False) returns an annotated RGB image
        annotated_img_rgb = results[0].plot(show=False)
        # Convert the annotated RGB image back to a ROS Image message
        output_msg = self.bridge.cv2_to_imgmsg(annotated_img_rgb, encoding='rgb8')
        # Publish the detection results
        output_msg.header = msg.header # Copy Timestamp information
        self.publisher_.publish(output_msg)

        # Check execution time
        if (self.detection_interval < (time.time() -self.last_detection_time)):
            # The detection process itself took a longer time than allocated detection_interval
            # The hardware is not capable enough to run detection at given rate
            self.get_logger().warn(
                "Possible : Rate limit too high! Should be less than : %.2f Hz"%(
                    (1/(time.time()-self.last_detection_time))))

def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YOLODetector()
    rclpy.spin(yolo_detector)  
    #The spin function runs a background process which checks for new incoming messages
    rclpy.shutdown()


if __name__ == '__main__':
    main()