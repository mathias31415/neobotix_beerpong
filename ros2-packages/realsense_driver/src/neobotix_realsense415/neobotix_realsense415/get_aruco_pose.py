import rclpy
from neobotix_realsense415.capture_image_client import CameraClient
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from scipy.interpolate import interp1d
import tf_transformations

calibration_matrix_path = "/home/docker_realsense/ros2_ws/src/neobotix_realsense415/neobotix_realsense415/CameraIntrinsics/calibration_matrix_0530.npy"
distortion_coefficients_path = "/home/docker_realsense/ros2_ws/src/neobotix_realsense415/neobotix_realsense415/CameraIntrinsics/distortion_coefficients_0530.npy"


def pose_estimation(frame, matrix_coefficients, distortion_coefficients):
    """
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    """

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)  # Aruco type can be modified here (DICT_4X4_250; ...)
    parameters = cv2.aruco.DetectorParameters()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters
    )

    pose_list_raw = []      # List to store the pose of the markers (raw, uncalibrated extrinsics from camera image)
    pose_list = []          # calibrated after offset and lookup table
    # If markers are detected
    if ids is not None:
        for i in range(len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i], 0.047, matrix_coefficients,
                distortion_coefficients
            )
            # print(f"rvec: {rvec}, tvec: {tvec}")

            # Store the ID and pose in a list
            pose_list_raw.append({
                'id': ids[i][0],
                'x': tvec[0][0][0],
                'y': tvec[0][0][1],
                'z': tvec[0][0][2],
                'r1': rvec[0][0][0],
                'r2': rvec[0][0][1],
                'r3': rvec[0][0][2]
            })

            # interpolate calibrated pose from lookup_table (measured on our zero_calibration_position aruco x=0, y_0, z=0.4)
            x_offset = 0.199
            y_offset = 0.1782
            z_offset = -0.2989

            x_raw=tvec[0][0][0]+x_offset
            y_raw=tvec[0][0][1]+y_offset
            z_raw=tvec[0][0][2]+z_offset
            angle_raw=rvec[0][0][1]

            x_lookup_offset,y_lookup_offset,z = lookup_points(z_raw)
            angle = lookup_angle(angle_raw) # only lookup rvec[0][0][1] --> rot aroung marker z --> camera has to be in horizontal plane (parralel to marker) when caputring image!!!

            x = x_raw+x_lookup_offset
            y = y_raw+y_lookup_offset
            z = z.item()
            angle=angle.item()

            # Output content -> quaternion hardcoded except of angle (calculated from rvec[0][0][1] --> only valid assumption if the camera is parallel to the marker plane/ horizontal)
            pose_list.append({
                'id': ids[i][0],
                'x': -x,
                'y': -y,
                'z': z,
                'qx': 0,
                'qy': 0,
                'qz': 1,
                'qw': angle,
            })


            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Draw Axis
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)

    else:
        print("Markers not detected")

    return frame, pose_list_raw, pose_list

def lookup_points(point_z):
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

def lookup_angle(angle):
     # Sensor output values (incorrect - 30 deg steps)
    sensor_output_angle = np.array([3.198, 2.976, 2.538, 1.959, 1.279, 0.492, -0.326, -1.104, -1.861, -2.463, -2.796, -3.040])    # vorzeichenfehler bei -90° -180° --> bereiche vermeiden!
    # Reference values (correct)
    reference_angle = np.array([-2.967, -2.443, -1.92, -1.396, -0.873, -0.349, 0.175, 0.698, 1.222, 1.745, 2.269, 2.793])

    # Create interpolation functions for each coordinate
    interp_angle = interp1d(sensor_output_angle, reference_angle, kind='linear', fill_value="extrapolate")
    angle = interp_angle(angle)

    return angle


def main(args=None):
    rclpy.init(args=args)       # initialize ros communications for a given context 

    camera = CameraClient()  # init the clients for communicating with the camera, start camera client node
    bridge = CvBridge()    # import cvBridge (converts image ros msg to cv2 readable format)


    # get the image from the service client and convert into cv2 format
    image_msg =  camera.capture_image()
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

    #load the camera calibration parameters
    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    # Perform pose estimation
    out_image, out_pose_list_raw, out_pose_list = pose_estimation(cv_image, k, d)

    # Print the pose list
    print("Pose List RAW:",out_pose_list_raw)

    print("Pose List:",out_pose_list)

    # Display the resulting frame
    cv2.imshow('Estimated Pose', out_image)
    cv2.waitKey(5000)

    camera.destroy_node()   # destroy camera client node
    rclpy.shutdown()    # shutdown previously initialized context
