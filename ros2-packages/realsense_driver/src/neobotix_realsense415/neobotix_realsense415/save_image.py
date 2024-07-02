import rclpy
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
    #cv2.imshow("Received Image", cv_image)
    #cv2.waitKey(2000)


    return cv_image

def save_image(img, filepath):
    try:
        # Save image to calibration folder
        cv2.imwrite(filepath, img)
        print("image saved sucessfully to "+ filepath)
    except Exception as e:
        print(f"An error occurred: {e}")


def main(args=None):      
    rclpy.init(args=args)       # initialize ros communications for a given context 
    camera = CameraClient()  # init the clients for communicating with the camera, start camera client node
    bridge = CvBridge()    # import cvBridge (converts image ros msg to cv2 readable format)

    dirpath = 'src/neobotix_realsense415/neobotix_realsense415/TestImagesArucoCalibration'
    for i in range(30):  
        # concenate filepath
        filename = f'testimage{i}.jpg'
        filepath = os.path.join(dirpath, filename)

        # capture image
        img = capture_image(camera, bridge)

        # save image
        save_image(img, filepath)

        # wait for next frame
        time.sleep(5)

    camera.destroy_node()   # destroy camera client node
    rclpy.shutdown()    # shutdown previously initialized context



if __name__ == '__main__':
    main()