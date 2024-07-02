import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from camera_interfaces.srv import GetArucoPose  # import the custom interface
import tf_transformations
from scipy.interpolate import interp1d
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster



class GetArucoPoseService(Node):

    def __init__(self):
        super().__init__('get_aruco_pose_server')    # create a node
        self.srv = self.create_service(GetArucoPose, 'get_aruco_pose', self.get_aruco_pose_callback)  # create service server
        self.subscription = self.create_subscription( Image, '/camera/color/image_raw', self.image_callback, 10)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self) # create a static transform broadcaster (for the aruco pose)
        self.latest_image = None
        self.bridge = CvBridge()    # init the converter to get cv2 image from message
        self.get_logger().info('Get Aruco Pose Service is ready.')

        # Path to the calibration matrix and distortion coefficients (generated with the calibration.py script)
        self.calibration_matrix_path = "/home/docker_realsense/ros2_ws/src/neobotix_realsense415/neobotix_realsense415/CameraIntrinsics/calibration_matrix_0530.npy"
        self.distortion_coefficients_path = "/home/docker_realsense/ros2_ws/src/neobotix_realsense415/neobotix_realsense415/CameraIntrinsics/distortion_coefficients_0530.npy"

        # Load calibration data
        self.matrix_coefficients = np.load(self.calibration_matrix_path)
        self.distortion_coefficients = np.load(self.distortion_coefficients_path)

    def image_callback(self, msg):  # define the callback function for the image subscriber
        self.latest_image = msg

    def get_aruco_pose_callback(self, request, response):    # define the callback function for the service server
        if self.latest_image is not None:
            self.get_logger().info('Image captured sucessfully.')

            try:
                camera_img = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")  # convert ROS Image message to OpenCV image
            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge Error: {e}')
                response.found = False
                return response

            gray = cv2.cvtColor(camera_img, cv2.COLOR_BGR2GRAY)
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)  # Aruco type can be modified here (DICT_4X4_250; ...)
            parameters = cv2.aruco.DetectorParameters()

            corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
                gray, aruco_dict, parameters=parameters
            )

            # Check if request.id is in marker ids
            if ids is not None and request.aruco_id in ids:
                # Find the index of the marker with the requested id
                marker_index = np.where(ids == request.aruco_id)[0][0]
                # Estimate pose of the marker with the requested id
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[marker_index], 0.047, self.matrix_coefficients,
                    self.distortion_coefficients
                )

                # Apply Extrinsic Calibration and Lookup-Table
                # interpolate calibrated pose from lookup_table (measured on our zero_calibration_position aruco x=0, y_0, z=0.4)
                # approx. camera to TCP
                x_offset = 0.199
                y_offset = 0.1782
                z_offset = -0.2989

                x_raw=tvec[0][0][0]+x_offset
                y_raw=tvec[0][0][1]+y_offset
                z_raw=tvec[0][0][2]+z_offset
                angle_raw=rvec[0][0][1]

                x_lookup_offset,y_lookup_offset,z = self.lookup_points(z_raw)
                angle = self.lookup_angle(angle_raw) # only lookup rvec[0][0][1] --> rot aroung marker z --> camera has to be in horizontal plane (parralel to marker) when caputring image!!!

                x = x_raw+x_lookup_offset
                y = y_raw+y_lookup_offset
                z = z.item()
                angle=angle.item()

                # Write response message
                response.found = True
                    # The frame id in the header is used as the reference frame of this transform.
                response.transform_stamped.header.frame_id = "ur_tcp_link"
                    # The frame id of the child frame to which this transform points.
                response.transform_stamped.child_frame_id = f"aruco_{request.aruco_id}"

                response.transform_stamped.header.stamp =  self.get_clock().now().to_msg()         # timestamp for publishing the tf to the tf tree topic

                #Translation (turn x and y because z axis is turned 180deg)
                response.transform_stamped.transform.translation.x = x
                response.transform_stamped.transform.translation.y = y
                response.transform_stamped.transform.translation.z = z

                # Convert the rotation vector to quaternion (quaternion hardcoded except of angle (calculated from rvec[0][0][1] --> only valid assumption if the camera is parallel to the marker plane/ horizontal)
                response.transform_stamped.transform.rotation.x = 0.0
                response.transform_stamped.transform.rotation.y = 0.0
                response.transform_stamped.transform.rotation.z = 1.0
                response.transform_stamped.transform.rotation.w = angle

                self.get_logger().info(f"Pose of marker {request.aruco_id} captured successfully.")
                self.get_logger().info(f"Marker {request.aruco_id} Pose is only valid, if camera is parllel to marker plane (e.g. horizontal to table!).")
            
                # Publish the transform
                self.tf_static_broadcaster.sendTransform(response.transform_stamped)
                self.get_logger().info(f"Transform of marker {request.aruco_id} published to tf_static topic sucessfully.")

            # requested marker ID not found
            else:
                self.get_logger().warn(f"Marker {request.aruco_id} not found.")

                response.found = False

        # no image available       
        else:
            self.get_logger().warn('No image available.')

            response.found = False

        return response
    

    
    def lookup_points(self,point_z):
        # Sensor output values (incorrect)
        sensor_output_z = np.array([0.1722, 0.2481, 0.3295, 0.4000, 0.4867, 0.5547, 0.6497, 0.7144])

        # Reference values (correct)
        reference_x_offset = np.array([-0.075, -0.05, -0.025, 0, 0.025, 0.05, 0.08, 0.1])
        reference_y_offset = np.array([-0.031, -0.021, -0.011, 0, 0.005, 0.015, 0.023, 0.032])
        reference_z = np.array([0.25, 0.3 ,0.35, 0.4, 0.45, 0.5, 0.55, 0.6])

        # Create interpolation functions for each coordinate
        interp_x_offset = interp1d(sensor_output_z, reference_x_offset, kind='linear', fill_value="extrapolate")
        interp_y_offset = interp1d(sensor_output_z, reference_y_offset, kind='linear', fill_value="extrapolate")
        interp_z = interp1d(sensor_output_z, reference_z, kind='linear', fill_value="extrapolate")

        x_offset = interp_x_offset(point_z)
        y_offset = interp_y_offset(point_z)
        z = interp_z(point_z)

        return x_offset, y_offset, z
    
    
    def lookup_angle(self,angle):
        # Sensor output values (incorrect - 30 deg steps)
        sensor_output_angle = np.array([3.198, 2.976, 2.538, 1.959, 1.279, 0.492, -0.326, -1.104, -1.861, -2.463, -2.796, -3.040])    # vorzeichenfehler bei -90° -180° --> bereiche vermeiden!
        # Reference values (correct)
        reference_angle = np.array([-2.967, -2.443, -1.92, -1.396, -0.873, -0.349, 0.175, 0.698, 1.222, 1.745, 2.269, 2.793])

        # Create interpolation functions for each coordinate
        interp_angle = interp1d(sensor_output_angle, reference_angle, kind='linear', fill_value="extrapolate")
        angle = interp_angle(angle)

        return angle


def main(args=None):
    rclpy.init(args=args)
    my_get_aruco_pose_service = GetArucoPoseService()
    rclpy.spin(my_get_aruco_pose_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
