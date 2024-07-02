import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from camera_interfaces.srv import CaptureImage  # import the custom interface

class ImageCaptureService(Node):

    def __init__(self):
        super().__init__('image_capture_server')
        self.srv = self.create_service(CaptureImage, 'capture_image', self.capture_image_callback)  # create service server
        self.subscription = self.create_subscription( Image, '/camera/color/image_raw', self.image_callback, 10)
        self.latest_image = None
        self.bridge = CvBridge()    # init the converter to get cv2 image from message
        self.get_logger().info('Image capture service is ready.')

    def image_callback(self, msg):  # define the callback function for the image subscriber
        self.latest_image = msg

    def capture_image_callback(self, request, response):    # define the callback function for the service server
        if self.latest_image is not None:
            response.image = self.latest_image
            self.get_logger().info('Image captured sucessfully.')
        else:
            self.get_logger().warn('No image available.')
        return response

def main(args=None):
    rclpy.init(args=args)
    image_capture_service = ImageCaptureService()
    rclpy.spin(image_capture_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
