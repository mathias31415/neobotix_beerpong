import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber_node')    # init node class, name node
        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.listener_callback, 10) # init topic subscriber
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()    # import cvBridge (converts image ros msg to cv2 readable format)

    def listener_callback(self, msg):
        self.get_logger().info('Receiving image')
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Display the image
            cv2.imshow("Received Image", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
