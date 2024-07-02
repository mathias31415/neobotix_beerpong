import numpy as np

def print_npy_contents(file_path):
    try:
        # Load the .npy file
        array = np.load(file_path)
        
        # Print array details
        print(f"Array data:\n{array}")
        print(f"Array shape: {array.shape}")
        print(f"Array dtype: {array.dtype}")
    
    except Exception as e:
        print(f"An error occurred: {e}")


def main():
    file_path_matrix = 'src/neobotix_realsense415/neobotix_realsense415/CameraIntrinsics/calibration_matrix.npy'
    file_path_distortion = 'src/neobotix_realsense415/neobotix_realsense415/CameraIntrinsics/distortion_coefficients.npy'

    print("-->  Camera Matrix:   ")
    print_npy_contents(file_path_matrix)

    print("-->  Distortion Coefficients:   ")
    print_npy_contents(file_path_distortion)


if __name__ == '__main__':
    main()