import cv2

def enumerate_cameras(max_cameras=10):
    """
    Enumerates available cameras by attempting to open them with OpenCV.

    Args:
        max_cameras (int): Maximum number of camera indices to check.

    Returns:
        List of available camera indices.
    """
    available_cameras = []
    for camera_index in range(max_cameras):
        cap = cv2.VideoCapture(camera_index)
        if cap.isOpened():
            available_cameras.append(camera_index)
            cap.release()  # Release the camera after testing
    return available_cameras

# Example usage
if __name__ == "__main__":
    cameras = enumerate_cameras()
    if cameras:
        print(f"Available cameras: {cameras}")
    else:
        print("No cameras found.")