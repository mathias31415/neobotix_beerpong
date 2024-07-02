import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from camera_interfaces.srv import CaptureImage  # import the custom interface


class CameraClient(Node):

    def __init__(self):
        super().__init__('image_capture_client')    # init node

        # init the service client
        self.capture_image_cli = self.create_client(CaptureImage, "/capture_image") # interface typ , service server name
        while not self.capture_image_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("capture_image service not available, waiting some more ...")
        self.get_logger().info("capture_image service available")


    def capture_image(self):    
        """
        None

        Returns
        -------
        sensor_msgs/Image
        bool success

        """
        future = CameraClient.send_request(self.capture_image_cli)
        response = self.wait_for_response(future)
        image = response.image
   
        return image
    

    @staticmethod
    def send_request(client):
        request = CaptureImage.Request()
        future = client.call_async(request)
        return future

    def wait_for_response(self, future):

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                    return None
                else:
                    return response



    def destroy_node(self) -> None:
        self.destroy_node()