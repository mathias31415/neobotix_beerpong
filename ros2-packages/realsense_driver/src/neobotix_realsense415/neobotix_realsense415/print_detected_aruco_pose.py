import rclpy
from neobotix_realsense415.get_aruco_pose_client import GetArucoPoseClient
from neobotix_realsense415.capture_image_client import CameraClient
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import os

def capture_image(camera, bridge):
    # get the image from the service clinet and convert into cv2 format
    image_msg =  camera.capture_image()
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

    # Display the captured image
    cv2.imshow("Received Image", cv_image)
    cv2.waitKey(2000)


    return cv_image


def main(args=None):      
    rclpy.init(args=args)       # initialize ros communications for a given context 
    detector = GetArucoPoseClient()  # init the clients
    #camera = CameraClient(),
    bridge = CvBridge()    # import cvBridge (converts image ros msg to cv2 readable format)

    # capture image and show
    #capture_image(camera, bridge)

    # get aruco pose estimation
    requested_id = 1
    transform_stamped, found = detector.get_aruco_pose(requested_id)
    print('###########################################################################')
    print("--> found Aruco Marker (true: 1/ false: 0) = ",found)
    print("--> Aruco Pose = ", transform_stamped)
    print('###########################################################################')


    detector.destroy_node()
    #camera.destroy_node()   # destroy camera client node
    rclpy.shutdown()    # shutdown previously initialized context



if __name__ == '__main__':
    main()