import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from camera_interfaces.srv import GetArucoPose  # import the custom interface


class GetArucoPoseClient(Node):

    def __init__(self):
        super().__init__('get_aruco_pose_client')    # init node

        # init the service client
        self.get_aruco_pose_cli = self.create_client(GetArucoPose, "/get_aruco_pose") # interface typ , service server name
        while not self.get_aruco_pose_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("get_aruco_pose service not available, waiting some more ...")
        self.get_logger().info("get_aruco_pose service available")


    def get_aruco_pose(self, id):    
        """
        # Request
        int8 aruco_id
        ---
        # Response
        geometry_msgs/TransformStamped transform_stamped # Orientation of the aruco marker
        bool found # True if the aruco marker was found

        """
        request = GetArucoPose.Request()
        request.aruco_id = 1
        future = GetArucoPoseClient.send_request(self.get_aruco_pose_cli, request)
        response = self.wait_for_response(future)

        transform_stamped = response.transform_stamped
        found = response.found

        return transform_stamped, found

    @staticmethod
    def send_request(client, request):
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