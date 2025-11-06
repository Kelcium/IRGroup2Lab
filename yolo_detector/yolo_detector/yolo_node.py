import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class YOLODetector(Node):
    
    def __init__(self):
        super().__init__('yolo_detector')
        self.publisher_ = self.create_publisher(Image, '/image_output_topic', 10)
        self.subscriber = self.create_subscription(Image, '/image_input_topic', self.image_callback, 10) #calls image_callback when receive message
        self.get_logger().info("YOLO Detector Node Started!")

    def image_callback(self, msg):
        self.get_logger().info("Received an Image")
        # TODO : Implement YOLO Detection Code
    
def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YOLODetector()
    rclpy.spin(yolo_detector)  
    #The spin function runs a background process which checks for new incoming messages
    rclpy.shutdown()


if __name__ == '__main__':
    main()